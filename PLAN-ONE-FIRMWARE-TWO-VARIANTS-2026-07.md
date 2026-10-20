# Plan: one firmware, two build variants — camera + SLIPS (2026-07-30, rev 2)

**Decision record.** Option B chosen 2026-07-30: instead of two repos sharing a
library, THIS repo becomes a monorepo producing two firmwares from one tree —
`[env:camera]` (container classifier, the live fleet) and `[env:slips]`
(structural logger). Rationale: identical hardware (same FireBeetle2 ESP32-S3
N16R8, same modem wiring), one maintainer, no need for independent release
schedules — and the February fork already proved that two copies of this code
drift. Not a one-way door: the modularization below is exactly the work a
shared library would need, so lifting `lib/` out for a future different-hardware
product stays a one-day job.

Supersedes: `ESP32-S3-WROOM1-NB-SLPIS/PLAN-SHARED-NBIOT-LIB-2026-07.md` (rev 1,
adversarially reviewed — all 13 findings carried into this revision).
Grounding: three deep-read analyses of 2026-07-30 (SLPIS tree, CAM-AI coupling
map, platform/Node-RED readiness); claims cite what the code does today.

---

## 0. Target architecture

```
ESP32-S3-WROOM1-CAM-AI/          ← becomes the product monorepo (rename optional)
  lib/                           ← shared modules, compiled per env (PIO LDF)
    QuectelAT/      UART core, sendAT, baud ladder 9600↔115200
    QuectelNet/     registration (CEREG/URC), attach(NetConfig), PSM/eDRX
    QuectelTCP/     sockets: open(mode), sendBinary + QISEND-counter recovery,
                    text JSON reader, binary-safe qird/readExact/readHeaderLine
    NbTime/         CCLK parse, TZ fallback (runtime cfg), RTC clock + fw stamp
    NbOtaClient/    OTA_CHECK/OTA_BEGIN client, esp_ota_*, md5, static 512B buf
    NbConfigChannel/ command dispatch + CFG_APPLIED ack, settings-only check-in
    NbIdentity/     salted-MAC device id + NVS
  src/
    main.cpp                     ← thin wake-cycle skeleton (gate → sample/work
                                    → uplink → config poll → OTA flags → sleep)
    variant.h                    ← the interface each variant implements
    variant_camera/              ← camera, Edge Impulse ML, PIR, geofence
                                    policy, SleepGeoConfig, OLED
    variant_slips/               ← ADS1115 driver + calibration, SCL3300,
                                    DS18B20, GNSS-DDC reader, SlpisConfig
  platformio.ini                 ← [env:camera], [env:slips],
                                    [env:slips_calibration], [env:scl3300_cal]
  scripts/preflight.sh           ← `pio run -e camera -e slips` — both MUST
                                    build before any commit
```

**Variant interface (small on purpose):** `variantInit()`,
`variantEarlyGate()` (camera: geofence/depot routing; slips: no-op),
`variantAcquire()` (photo+ML / ADC sample set), `variantBuildTelemetry(kv&)`,
`variantHasBinaryPayload()` (+ payload accessor for the camera JPEG),
`variantRegisterConfigHandlers(NbConfigChannel&)`, `variantPrepareSleep()`.

**Per-env config:** one `FirmwareConfig` (server IP, telemetry port 8009, OTA
port 8010, `app:` wire tag, APN, identity salt, pacing) — today these are
constants scattered through two files; centralizing kills the silent-crosstalk
bug class. Edge Impulse SDK is a camera-env-only dependency
(`build_src_filter` + per-env `lib_deps`/`lib_ignore`).

**Versioning:** ONE `FW_VERSION` stream for the whole monorepo. Version N
builds `camera_vN.bin` and `slips_vN.bin`; each uploads to its own family
namespace on the server (Phase 1). Variants may ship no-op releases in
lockstep — that's fine and keeps the mental model trivial.

**Rules that keep a monorepo honest:**
- Variant code lives in variant files — `#ifdef` only at the `variant.h` seam.
- No commit without `scripts/preflight.sh` passing (both envs compile).
- Behavioral invariants travel verbatim and keep their war-story comments:
  250–300 ms uplink pacing, QISEND sent-counter recovery, buffer-mode-0
  sockets, post-NRB baud resync, static 512 B OTA buffer, 512 B QIRD cap.

---

## 1. Phase 0 — safety (0.5 day, first)

1. **SLPIS repo: commit the working tree.** ~3 months of work (+225/−47 in
   `main.cpp`: GNSS DDC parser, DS18B20, variant flags, SCL3300 power-down,
   PSM/eDRX tuning) exists ONLY uncommitted on a 2026-03-04 base. Commit on a
   branch (`pre-monorepo-snapshot`). Never `git pull upstream` there (upstream
   is this repo; a merge would be carnage).
2. **This repo: reconcile `FW_VERSION`** (ini says 9; the staged
   `firmware_v10.bin` came from the v10 tree) — re-bump, commit, tag the
   v10 state. The monorepo restructure will ship as v11.
3. **Pin `platform = espressif32`** to one exact version in both projects now,
   monorepo keeps the pin (unpinned core drift under shared code is the classic
   silent breakage; ArduinoJson & co. get explicit `lib_deps` entries).
4. Timestamped `.bak` before touching `TCP_flows.json` (standing convention).

---

## 2. Phase 1 — platform family-safety (3–5 days; gates ALL slips uploads)

Unchanged from rev 1 — the monorepo still produces two different, incompatible
binaries, so none of this softens.

### 2.1 OTA family separation (the dangerous one)

Confirmed today: `firmware_images` PK is a bare integer version with
`ON CONFLICT(version) DO UPDATE` (structural v12 would silently REPLACE camera
v12 in the DB **and** clobber `firmware_v12.bin` on disk); one fleet rollout
key; `resolveOtaForDevice()` never consults family. Cross-flash risk is real
(md5 matches, devices boot the wrong product).

- **Step 0 — backfill migration, gates the rest:** existing cameras were
  auto-registered with `app_id` NULL; a family gate without backfill silently
  kills the LIVE fleet's OTA (`no_family` looks like a clean decision).
  Snapshot the production DB → assign camera family to all existing `devices`
  and `firmware_images` rows → startup report of NULL-family devices → only
  then enable the NULL→`OTA_NONE` rule.
- Canonical family slugs (**D6**, blocks this phase): registry app for cameras
  is `waste-monitoring`, wire tag is `container-classifier` — pick one slug per
  family, document the mapping once.
- `firmware_images`: (family, version) PK — SQLite table rebuild
  (create/copy/rename) with a written rollback; update ALL version-keyed call
  sites (`listFirmwareImages`, `getFirmwareImage`, `getLatestFirmwareImage`,
  `deleteFirmwareImage`, delete endpoint).
- **On-disk scoping too**: `<family>/firmware_vN.bin` (or family-prefixed
  names); migration renames existing files.
- Per-family rollout keys (`ota_rollout_version:<family>`; legacy key aliases
  camera). `resolveOtaForDevice()`: device row → family → that family's images
  + rollout only.
- Upload API: required `X-Firmware-Family` header; admin UI groups images and
  rollout per family; `ota_events` logs family.
- **Wire hardening (new in rev 2, enabled by rebuilding firmware anyway):**
  firmware appends `fam=<slug>` to `OTA_CHECK`. Server: absent → legacy camera
  behavior (back-compat with fielded v8–v10); present → must match the
  registry family or reply `OTA_NONE` (`family_mismatch`, audited). Protects
  reflashed bench units whose registry entry wasn't switched.

### 2.2 Config channel: per-family vocabulary

Transport (pending/current blobs, internal command endpoints, CFG_APPLIED
fold) is already device-agnostic — keep. But the field set/validator/
camel→snake map are one flat camera-only list, unknown keys silently dropped,
and a NULL-family SLIPS unit lands in the *container* UI whose DeviceSettings
queues all 11 camera fields at it forever.

- Family-keyed field sets: camera unchanged (11 fields, same wire contract);
  structural first cut (**D3**): `sampleIntervalS` (60–86400),
  `wakeHour/Minute`, `napHour/Minute`, `adcGain` (enum), `sampleAvgCount`
  (1–64), `gnssEnable` (0/1). JSON-blob storage → adding fields later needs no
  migration.
- New `StructuralDeviceSettings` panel in `screens-structural.jsx` (the
  existing DisplacementBaselinePanel is a server-side display offset — keep
  separate). Auto-registration accepts an app hint; each family's UI renders
  only its own panel. Home-sanity guard stays camera-only.

### 2.3 Node-RED: one-socket telemetry + command pickup

Command-reply leg currently fires only on `settings_only:1` polls or after a
JPEG — a plain telemetry cycle (all SLIPS does) leaves configs pending
forever. Change: header carrying BOTH telemetry and `settings_only:1` emits
the telemetry message (output 2) AND the Fetch-Device-Command message
(output 3) instead of returning early.

Guards (the live camera depot poll rides this node): telemetry path only on
non-empty telemetry hex; hex decode stays in try/catch; both messages built
BEFORE `resetState()`. Three acceptance tests in `NODE-RED/test/` first:
settings-only WITHOUT telemetry (camera today — byte-identical), settings-only
WITH telemetry (slips), full photo cycle. Node-id drift (**D8**): repo
source-of-truth node is `a97faed237500581`; the deployed VM runs the same code
as node `5ad44a083607c498` — patch repo, test, then surgically patch the VM
node (runbook §2 procedure); reconcile the two before patching.

Known field-name hazards (bites **D4**): expansion regexes are unanchored —
`gnss_lat=`/`gnss_lon=` would be mangled to `gnss_latitude=`/`gnss_longitude=`
in flight, and the structural ingest whitelist has no `gnss_*` entries at all.
Enabling GNSS is firmware + flow + server, not a compile flag.

### 2.4 Small but real

- Per-family (or per-device) staleness thresholds in `computeDeviceIssues` —
  camera cadence (stale >6 h) would flag a slow-sampling SLIPS forever. Battery
  plumbing for structural already works (`updateDeviceHealth` with battery is
  called by the structural ingest) — verify on bench, don't re-implement.
- Scope: only §2.1 (with backfill) and §2.3 gate slips firmware work; the UI
  items can trail.

---

## 3. Phase 2 — restructure this repo into core + variants (2–3 days)

1. Extract the modules per the coupling map (~1,300–1,450 lines) into `lib/`,
   parameterizing: serial handle + pins/baud, APN + factory-reset flag + PSM
   timers (`NetConfig`), FW_VERSION + device id (params), the ~69
   `oled_status()` sites → one status callback (camera's OLED subscribes;
   slips ignores). Fix the one zero-delay poll loop in `sendATCommand` while
   moving it.
2. Distill `sendPhotoOverTCP` → `NbUplink`: header from key/value list +
   optional marker-framed binary body (camera JPEG) + pacing + reply-wait
   hook. Slips passes no body.
3. Split `parseAndApplyCommand`: dispatch + CFG_APPLIED confirmed-ack protocol
   → `NbConfigChannel` (contract frozen — the server's pending→current fold
   for both fleets depends on the exact ack semantics); `set_config` field
   handlers register per variant.
4. `variant_camera/` = current camera/ML/PIR/geofence/OLED code behind
   `variant.h`; `main.cpp` becomes the shared skeleton.
5. **NVS ownership map** (documented in the repo): `NbIdentity` owns
   `device_identity` (shared by design); `NbTime`'s RTC clock requires the
   app's sleep path to keep `ESP_PD_DOMAIN_RTC_SLOW_MEM` ON (hard requirement,
   both variants); `sleep_geo` is camera-variant-owned; slips config gets a
   NEW namespace — never reuse across variants (same hardware, NVS survives
   reflashes; a bench board that ran camera firmware carries camera blobs).
6. `scripts/preflight.sh` + (optional) a tiny CI job running it.
7. **Acceptance:** `[env:camera]` v11 passes bench HIL (wake gate, photo
   cycle, config round-trip, OTA check) with behavior identical to v10, then
   ships as an OTA canary to the field unit before any rollout. Modem-code
   freeze is trivial now (one tree), but keep the extraction→v11 window short.

---

## 4. Phase 3 — the slips variant (2–3 days + bench session)

Migrate from the SLPIS repo into `variant_slips/`, preserving verbatim the
field-proven parts: minimal ADS1115 register driver + calibration constants
(K=−38.501 build flags → `[env:slips_calibration]`), SCL3300 power-down frames
+ NVS zero-offset flow, DS18B20, 50 kHz single-clock I2C + bus recovery, NVS
single-slot telemetry retry, 999-sentinel telemetry schema, GNSS-DDC reader
(behind `gnssEnable`).

What the variant gains by riding the shared core (everything SLPIS is missing
today): 115200 baud ladder (it's at 9600, no IPR), buffer-mode-0 sockets (it
opens accessMode=1 — the silent-reply-loss gotcha), binary-safe reads, OTA
client, persistent remote config (today's `set_sleep` is deliberately
overwritten every timer wake — the NVS `SlpisConfig` makes sampling interval +
window real and power-cycle-safe), RTC clock + window gate, `fam=slips` in
OTA_CHECK.

Slips-specific steps:
1. **Partition table → `partitions_ota_16mb.csv`** — the one unavoidable USB
   reflash per existing unit (NVS offsets match; identity survives).
2. One-socket wake cycle: sample → uplink (telemetry + `settings_only:1`) →
   command reply → CFG_APPLIED → deferred OTA flags → PSM sleep.
3. Registration budget / mid-cycle abort (today a bad-coverage site can hold
   the device awake 10+ min); reconsider the inherited brown-out-disable on a
   battery device; README/salt cleanup per **D2**.
4. SLPIS repo: final commit "migrated to monorepo @ <sha>" + README pointer →
   archived read-only. Its git history stays there.
5. **Bench-unit personality switching:** reflashing a board to the other
   variant requires switching its registry family too, or OTA serves the wrong
   binary — the `fam=` mismatch guard turns that mistake into a logged
   `OTA_NONE` instead of a cross-flash.

---

## 5. Phase 4 — async where it pays (1–2 days, after parity)

1. Enforce the serial mutex (exists, never taken) inside the lib around every
   AT transaction — prerequisite for any concurrency.
2. `netTask` bring-up overlap (the real wins): slips samples sensors (and
   optionally acquires GNSS, 60 s budget) while the modem registers
   (~10–90 s); camera overlaps GNSS fix ∥ registration (saves 20–35 s on away
   cycles). App waits on an event group.
3. OTA + uplink stay synchronous (nothing else needs the CPU mid-download;
   task = stack risk for zero gain — the static 512 B buffer exists because of
   the 10 KB task stack).
4. WDT discipline: lib waits use `vTaskDelay`-based polling so enabling the
   task watchdog later doesn't fire on 120 s registration waits.

---

## 6. Phase 5 — prove it (1 bench day + field soak)

Build both envs at vN; upload each to its family; per-family bench HIL (wake
cycle, config pending→CFG_APPLIED→current fold, OTA vN→vN+1 through the real
chain); one canary per family for a full day; then rollout. Extend
`HIL-RUNBOOK-2026-07.md` with a structural section. Separate hardware track
(not this refactor): SLPIS sleep floor measured ~3.9 mA average (eDRX paging
spikes) vs ~65 µA design plateau — blocks multi-year battery claims, needs its
own PSM/eDRX investigation.

---

## 7. Decisions

- **D1 — SETTLED (this revision):** monorepo in this repo; optional cosmetic
  rename of the repo later.
- **D2 — identity salt:** slips still uses the camera salt. Identity is
  NVS-first, so rotating affects only fresh/erased units. Keep (cosmetic debt)
  vs rotate + erase `device_identity` on bench units (clean break)?
- **D3 — structural config vocabulary:** §2.2 first cut OK?
- **D4 — GNSS on slips:** worth the 60 s acquire budget? Requires the §2.3
  regex/whitelist work (was D7, folded in) — not a compile flag alone.
- **D5 — ordering:** Phase 1 before Phase 2/3 as written; parallelize only
  respecting the two hard rules: no slips uploads before §2.1+backfill lands,
  and keep the extraction→v11 window short.
- **D6 — family slugs + backfill policy (BLOCKS Phase 1):** canonical slug per
  family (`waste-monitoring` vs `container-classifier` mismatch), the policy
  for existing NULL-app devices, and the on-disk artifact naming in one go
  (pick either `<family>/firmware_vN.bin` subdirs or
  `firmware_<family>_vN.bin` prefixes; build outputs named to match).
- **D8 — Node-RED source of truth:** reconcile repo `a97faed2…` vs deployed
  `5ad44a08…` before the next surgical patch.
- **D9 — version pins:** exact `espressif32` platform version to freeze on.

## 8. Effort

| Phase | What | Est. |
|---|---|---|
| 0 | commits/tags/pins | 0.5 d |
| 1 | platform family-safety (backfill + OTA rebuild + config + Node-RED + thresholds + UI) | 3–5 d (UI trails) |
| 2 | monorepo restructure, camera v11 canary | 2–3 d |
| 3 | slips variant + bench | 2–3 d + bench |
| 4 | targeted async | 1–2 d |
| 5 | HIL + canary + rollout per family | 1 d + soak |

Total ≈ 9–13 working days to two firmwares from one tree, both remotely
updatable and configurable, with the platform understanding two device
families.

---

*Rev 1 (two repos + shared library) adversarially reviewed 2026-07-30, 13
findings applied. Rev 2 (this doc) re-architects around the monorepo decision;
all rev-1 platform findings carry over unchanged.*
