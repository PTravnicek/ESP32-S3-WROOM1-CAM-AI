# HIL runbook + production remediation (2026-07-27)

> **2026-07-29 — fw v9 TZ regression, fixed in v10.** Field data showed the
> device sleeping 20:00–04:00 local instead of the configured 22:00–06:00:
> both edges shifted exactly −2 h. Root cause: the Czech NB-IoT operator's
> CCLK is **local time with no TZ suffix**, not UTC as §7.2 of the old handoff
> assumed — v9's EU-DST auto fallback added CEST on top of already-local time.
> v10 (md5 `652377aaa715d500746d3a1af8f02d18`) reverts the default to
> "suffix-less CCLK is already local" (fw ≤v8 semantics; DST logic stays
> available behind `FALLBACK_TZ_AUTO_CZ=1` for genuinely-UTC networks) and
> adds a version stamp to the RTC-persisted clock so any reflash forces one
> modem re-sync — without that, the +2 h clock would have survived the v10
> flash in RTC memory. Use **v10** everywhere §4 below says v9. Since v9
> reached the device over USB (ota_events shows no transfer), the v9→v10
> update doubles as the first true remote-OTA proof.

Continuation of `HANDOFF-DEVICE-CONFIG-HIL-2026-07.md`. This file records what
was verified on 2026-07-27 without hardware, what was fixed, and the exact
steps left to run — ordered so that a device can be shipped to a client with
confidence.

## 0. State as of 2026-07-27 evening

Verified working (no hardware needed):

- **Config channel E2E on production**: simulator (`NODE-RED/test/device-sim.js`)
  ran a settings-only poll and a full 12 KB photo cycle against
  `34.10.203.180:8009` with test UID `c6daa2e684d55b02` — reply arrived on the
  same socket, JPEG validated, image written under
  `/var/lib/node-red/images/c6daa2e684d55b02/`.
- **Pending-config lifecycle** (fetch → identical re-send while unacked → no
  ack keeps pending → ack folds pending→current) proven against the real
  `server.js`+`db.js` (local instance; code is byte-identical to what runs on
  the VM — md5-verified).
- **Node-RED accumulator**: 18-case unit suite (`NODE-RED/test/run-tests.sh`),
  covering photo/settings/ack cycles, chunked delivery, session isolation,
  context sweep, corrupt + duplicated-block streams.
- **fw v9 builds clean** (`pio run` SUCCESS); `firmware_v9.bin` staged in the
  project root, md5 `879399fa3f4e279c70863979f6d7adeb`.
- Field device `649195f647c447b3` is LIVE on fw v8 (flashed over USB
  2026-07-27), checking in roughly hourly over NB-IoT, and its OTA_CHECK is
  reaching port 8010.

Fixed on the VM today:

- Disk was 96 % → journal vacuum freed 2.5 GB → **87 %, 3.8 GB free**.
- `OTA_ENABLED=0` appended to `/opt/iot-dashboard-staging/.env` (takes effect
  on the staging restart in §1).

Fixed in the local tree (deploy needed):

| Fix | Where | Deploy via |
|---|---|---|
| Accumulator: salvage now sees the full stream (hold-for-displaced-tail + END-marker trim in dedup) | `NODE-RED/TCP_flows.json` (node `a97faed2`), tested copy in `NODE-RED/test/accumulator-under-test.js` | §2 below |
| Accumulator: session-context leak fix (was written 07-02 but never deployed) | same | §2 below |
| Home-coordinate sanity guard (409 `HOME_FAR_FROM_DEVICE` + `confirmFarHome` override; only fires when home actually moves >50 m and geofence enabled; fail-open on Influx outage) | `platform/server.js`, UI confirm flow in `public/app/` | §3 below |
| Pending-config age: `device_config_pending_at` column, `deviceConfigPendingAt` in GET config, `config_undelivered` warning in device issues after 48 h | `platform/lib/db.js`, `server.js` | §3 below |
| TZ fallback: EU DST rule (CET/CEST from UTC date) when CCLK lacks a TZ suffix — `FALLBACK_TZ_AUTO_CZ=1`, math unit-tested incl. 2026/2027 switch dates | `src/main.cpp`, **FW_VERSION=9** | §4/§5 below |

Still broken until §1 is run:

- **Prod OTA listener is dead** (`EADDRINUSE` since the Jul 15 restart) — the
  *staging* process has held port 8010 since Jun 19. Devices' OTA checks are
  answered from the **staging DB** (rollout v3, only firmware_v3.bin). Any
  rollout set in the prod admin UI does nothing until prod owns 8010.

Known-but-deliberate:

- Depot-poll power cost (§7.3 of the handoff): *measure before throttling* —
  procedure in §6.3.
- Deployed Node-RED flow still carries an orphaned duplicate chain (old
  accumulator + fetch/delivered pointing at port **3000**/Grafana). It receives
  no traffic (tcp-in feeds only `5ad44a08`). Delete it in the editor when
  convenient — never wire anything into it.
- The `/api/internal/*` endpoints are unauthenticated but port 3001 is not
  reachable from the internet (GCP firewall verified). Defense-in-depth
  (internal API key) is a nice-to-have, not urgent.

## 1. Free port 8010 for prod OTA  (5 min, do first)

```bash
ssh pavel_travnicek_89_gmail_com@iot-secure
sudo systemctl restart iot-dashboard-staging   # picks up OTA_ENABLED=0, releases 8010
sleep 3
sudo systemctl restart iot-dashboard           # prod OTA server now binds 8010
sleep 3
sudo ss -tlnp | grep ':8010'                   # must show the pid of iot-dashboard (MainPID below)
systemctl show iot-dashboard -p MainPID
sudo journalctl -u iot-dashboard -n 50 | grep -i '\[ota\]'   # expect "listening", no EADDRINUSE
printf 'OTA_CHECK device_id=649195f647c447b3 ver=8\n' | nc -w 5 localhost 8010
# expect: OTA_NONE  (prod has no rollout yet) — and the request now appears in
# the PROD journal, not iot-dashboard-staging's
```

## 2. Deploy the fixed accumulator to Node-RED  (10 min)

The **active** accumulator on the VM is node id `5ad44a083607c498`
(tcp-in `:8009` wires only to it). The local flow file's newest code lives
under a *different* id (`a97faed237500581`) — a plain re-import would duplicate
chains again (that is exactly how the orphan chain happened). Patch the active
node's function in place:

```bash
cd ~/AI_BUSINESS_FUN
NODE-RED/test/run-tests.sh    # must print "18 passed, 0 failed" before deploying
scp NODE-RED/test/accumulator-under-test.js pavel_travnicek_89_gmail_com@iot-secure:/tmp/acc-new.js
ssh pavel_travnicek_89_gmail_com@iot-secure
sudo cp /var/lib/node-red/flows.json /var/lib/node-red/flows.json.bak-$(date +%Y%m%d-%H%M%S)
sudo python3 - <<'EOF'
import json
src = open('/tmp/acc-new.js').read()
p = '/var/lib/node-red/flows.json'
flows = json.load(open(p))
hits = [n for n in flows if n.get('id') == '5ad44a083607c498' and n.get('type') == 'function']
assert len(hits) == 1, hits
hits[0]['func'] = src
json.dump(flows, open(p, 'w'), indent=1)
print('patched, func len', len(src))
EOF
sudo systemctl restart node-red
sudo journalctl -u node-red -n 20 --no-pager    # flow starts clean
```

Verification: on the next device check-in (hourly) expect the usual
`*** JPEG valid ***` cycle; after ~2 devices have connected, `Context sweep:`
lines confirm the leak fix is live. From the laptop:
`node NODE-RED/test/device-sim.js photo` should behave exactly as before.

## 3. Deploy the platform changes  (5 min)

**One-time prerequisite (found 2026-07-28):** `/opt/iot-dashboard` is owned by
uid 501 — an artifact of old `USE_GCLOUD` deploys that extracted with
`sudo tar`. The direct-ssh rsync path runs as the ssh user and gets
`mkstemp: Permission denied`, so the deploy silently ships nothing. The
service itself runs as `User=pavel_travnicek_89_gmail_com`, so that user is
the correct owner on every axis. Fix once:

```bash
ssh pavel_travnicek_89_gmail_com@iot-secure 'sudo chown -R pavel_travnicek_89_gmail_com /opt/iot-dashboard'
```

Then deploy (verify afterwards that the rsync output shows no
"Permission denied" lines):

```bash
cd ~/AI_BUSINESS_FUN
SKIP_TRAFFIC_AUTOMATION=1 ./deploy-iot-magic.sh pavel_travnicek_89_gmail_com@iot-secure
```

Migration runs automatically on service start (`device_config_pending_at`).
Smoke test the new guard from /app on the **bench** device `c6daa2e684d55b02`:
set Home to e.g. Brno while its last fix is České Budějovice → expect the
"home is N km from device" confirm dialog; cancel, then save unchanged values
→ no dialog (guard only fires when home moves).

## 4. Prove remote OTA v8 → v9 on the live field device  (the big one)

`649195f647c447b3` runs v8 in the field and checks OTA hourly. After §1:

1. Admin UI → **Firmware / OTA** → upload `firmware_v9.bin`
   (project root; md5 `879399fa3f4e279c70863979f6d7adeb`, version **9**).
   *(Upload alone changes nothing on devices.)*
   *(2026-07-28: an initial upload attempt got HTTP 413 — nginx's default 1 MB
   body limit; fixed with `client_max_body_size 32m` in
   `/etc/nginx/sites-enabled/iot-dashboard-main`, backup at
   `/etc/nginx/iot-dashboard-main.bak-20260728-091640`. Note this site file,
   not `sites-available/iot-magic.conf`, is the live config for iot-magic.com.)*
2. **Pin** device `649195f647c447b3` to target version 9 (canary; fleet
   rollout stays untouched).
3. Watch: `sudo journalctl -u iot-dashboard -f | grep -i '\[ota\]'`
   — next check-in should log `OTA_BEGIN size=2291296 ver=9` and a transfer.
   At 115200 baud the ~2.3 MB download takes ~4 min of device awake time.
4. Next cycle after that must report `ver=9` → `OTA_NONE (up_to_date)`, and
   `[SCHED]` lines in its serial output (if ever attached) would show the DST
   fallback. Check `ota_events` for the transaction record.
5. Only then consider setting the fleet **rollout** to 9.

If the download stalls or the device bootloops → physical access is the
fallback (pin it back to 8 first so a recovered device does not re-download 9).

## 5. Live config-channel HIL on the field device  (no USB needed)

The device polls settings on every home wake and after every photo upload, so
config delivery can be proven end-to-end from the couch:

1. /app → device `649195f647c447b3` → Device settings → change **Sleep (s)
   away** 3600 → 3300 (harmless, easy to spot) → Save. Badge shows "queued".
2. `sudo journalctl -u node-red -f` — on the next check-in expect
   `Device command response: {"cmd":"set_config",...}` then
   `CFG_APPLIED ack -> delivered ... {"success":true}`.
3. GET config (or reload /app): pending gone, current shows 3300, and the
   check-in interval visibly shortens.
4. Revert to 3600 the same way. Both directions prove idempotent delivery.
5. Adverse-radio variant without hardware (needs a pending config on the
   BENCH uid): seed a pending for `c6daa2e684d55b02` (one UI save), then
   `node NODE-RED/test/device-sim.js settings --kill-before-ack` → config must
   STAY pending (re-offered), then `node NODE-RED/test/device-sim.js settings`
   → applied + folded.

## 6. USB HIL (when a device is on the bench)

Flash: `~/.platformio/penv/bin/pio run -t upload` then
`pio device monitor` — first line must say `[FW] build fw v9`.

### 6.1 Window gate, three time sources (handoff §8.2 #1)

- **Cold boot, no clock**: power-cycle (not reset) outside the window
  (set nap = now−10 min via /app first). Expect
  `No clock yet — registering modem…` → `Night [modem(cold)] — single …s deep
  sleep`. With `TZ: fallback` in the log the new DST math is in play — verify
  the printed local time matches wall clock (CEST = UTC+2 until Oct 25 2026).
- **RTC-persisted wake**: inside the window, let it complete one cycle, then
  on the next timer wake expect `persisted RTC clock, no radio` and NO
  registration lines before the gate decision.
- **Night decision**: set nap a few minutes ahead; the following wake must
  sleep without camera/ML.

### 6.2 Config under adverse radio (handoff §8.2 #3)

Pull the antenna (or shield the modem) after the reply is requested but before
ack; config must stay pending and land on the next full cycle. The
socket-level equivalent is already covered by §5.5.

### 6.3 Depot poll power cost (handoff §8.2 #2 — measure BEFORE throttling)

Bench at "home" (set homeLat/Lon to bench, radius 500), `sleepHomeS=600`,
current probe on the battery. Record per-wake: total awake seconds and the
share spent in `[CFG] settings-only poll` (registration dominates). Compare a
PSM warm wake against a cold boot — if warm wakes keep the registration
(`Module already registered! Minimal setup`), the poll is cheap and needs no
throttle; if every wake pays ~90 s of radio, throttle before shipping
(e.g. poll every Nth home wake).

### 6.4 Image-integrity soak (handoff §8.2 #4)

Leave the device cycling with `sleepAwayS=60`, radius 0 (always away).
On the VM: `sudo journalctl -u node-red -f | grep -E 'Tail diag|REJECTED|SALVAGE'`.
Healthy = every finalize shows the 4-byte END_MARKER trailer. `SALVAGE` lines
mean the modem duplicated a block and the new recovery path saved the photo —
worth counting but not a failure. `REJECTED` + `_debug/*.raw` = regression.

## 7. Ship checklist for a client unit

- [ ] §1–§3 done; §4 proven (remote OTA is the no-truck-roll safety net)
- [ ] Unit flashed v9, `[FW] build fw v9` seen on serial
- [ ] Registered in /app with client assigned, home = client depot,
      radius sane, window per contract; config **delivered** (not pending)
- [ ] One full photo cycle observed from the unit (JPEG valid, telemetry in
      dashboard, reply received)
- [ ] OTA: unit reports current version, `ota_enabled=1`, no pin left behind
- [ ] `config_undelivered` / offline alerts visible to whoever will operate it
- [ ] Firmware source of the shipped build committed/tagged
      (`src/main.cpp.v8-as-flashed-20260727` preserves the v8 field build;
      v9 = current tree)

## 8. File map (new/changed today)

| What | Where |
|---|---|
| Accumulator unit tests | `~/AI_BUSINESS_FUN/NODE-RED/test/{run-tests.sh,test-accumulator.js}` |
| Device simulator | `~/AI_BUSINESS_FUN/NODE-RED/test/device-sim.js` |
| Fixed accumulator (source of truth) | `~/AI_BUSINESS_FUN/NODE-RED/TCP_flows.json` node `a97faed2…` (backup `TCP_flows.json.bak-salvagehold-*`) |
| Platform guard + pending-age | `platform/server.js`, `platform/lib/db.js`, `platform/public/app/{live-data.js,app.jsx,screens-container.jsx}` |
| fw v9 (DST fallback) | `src/main.cpp` (v8 snapshot: `src/main.cpp.v8-as-flashed-20260727`), `platformio.ini` |
| v9 binary for OTA upload | `firmware_v9.bin` (md5 `879399fa3f4e279c70863979f6d7adeb`) |
