# Hand-off: device config channel + hardware-in-the-loop testing (July 2026)

> **2026-07-27 update:** continued in `HIL-RUNBOOK-2026-07.md` — off-device
> verification done (accumulator test suite, device simulator, E2E vs prod),
> §7 gaps 1/2/4/5 addressed, fw is now **v9**, and a production OTA
> port conflict was found (staging held :8010). Read the runbook first.

How the **remote device-configuration channel** works end to end, how to build,
flash and deploy each piece, what a healthy run looks like on the wire, and what
is still broken. Written as the briefing for whoever designs and runs the
hardware-in-the-loop (HIL) test next.

Current state: **fw v8**, compiles clean (`pio run` → SUCCESS). The full chain
(web app → server → Node-RED → device → ack) has been observed working on real
hardware over NB-IoT. Sections 7 and 8 list what has *not* been proven and what
is known broken — read those before designing tests.

---

## 0. TL;DR

An operator edits device settings in the web app. The change is stored
server-side as **pending**, handed to the device on its next check-in, applied
to NVS, and acknowledged back — only then does the server mark it **current**.

```
/app/ device detail  --PATCH-->  platform API (SQLite: pending)
                                      |
device wakes, opens TCP :8009 --> Node-RED --GET--> /api/internal/device-command/<uid>
                                      |
        reply {"cmd":"set_config",...} back down the SAME socket
                                      |
device applies + saves NVS, writes "CFG_APPLIED\n" --> Node-RED POSTs /delivered
                                      |
                          pending folds into current
```

Losing any step is safe: the config stays pending and is re-sent next cycle.
Re-applying the same config is idempotent.

---

## 1. The three moving parts

| Part | Location | Deployed how |
|---|---|---|
| Firmware | `~/Documents/PlatformIO/Projects/ESP32-S3-WROOM1-CAM-AI/src/main.cpp` | `pio run -t upload` over USB |
| Platform (API + web app) | `~/AI_BUSINESS_FUN/platform/` | `./deploy-iot-magic.sh` (see §4) |
| Node-RED flow | `~/AI_BUSINESS_FUN/NODE-RED/TCP_flows.json` | Import in Node-RED UI → Deploy |

Production VM: `pavel_travnicek_89_gmail_com@iot-secure` (GCE, external
`34.10.203.180`). Node-RED editor: `https://node-red.iot-magic.com`.
Web app: `https://iot-magic.com/app/`.

**Critical infrastructure fact:** the platform Node app listens on
**127.0.0.1:3001** on the VM. **Port 3000 is Grafana** — it answers with
`{"message":"Not found"}`, which looks exactly like an API 404 and has already
cost a full debugging session. All Node-RED → platform calls must use 3001.

---

## 2. Config schema

Canonical field set. The API/UI uses camelCase; the firmware command uses
snake_case; both map to one NVS struct (`SleepGeoConfig` in `main.cpp`).

| API (camelCase) | Wire (snake_case) | Range | Meaning |
|---|---|---|---|
| `wakeHour` / `wakeMinute` | `wake_hour` / `wake_minute` | 0–23 / 0–59 | Start of operational window (local) |
| `napHour` / `napMinute` | `nap_hour` / `nap_minute` | 0–23 / 0–59 | End of operational window |
| `homeLat` / `homeLon` | `home_lat` / `home_lon` | ±90 / ±180 | Depot centre |
| `homeRadiusM` | `home_radius_m` | 0–2 000 000 | Depot radius; **0 disables the geofence** |
| `sleepHomeS` / `sleepAwayS` | `sleep_home_s` / `sleep_away_s` | 60–86400 | Deep-sleep interval per state |
| `pirHomeMode` / `pirAwayMode` | `pir_home_mode` / `pir_away_mode` | 0/1/2 | 0 = PIR+timer, 1 = timer only, 2 = PIR only (timer capped 24 h) |

Partial updates are allowed — send only changed fields; the server merges over
current+pending. Every field is range-validated server-side **and** clamped
again in firmware.

---

## 3. Wire protocol (exact)

### 3.1 Normal check-in (device has a photo)

Device opens TCP to `34.10.203.180:8009` and sends:

```
CORRELATION_HEADER\n
app:container-classifier\n
device_id:<hex16>\n
measurement_id:<uuid>\n
timestamp:<millis>\n
image_size:<bytes>\n
telemetry:<hex-encoded influx line protocol>\n
END_HEADER\n
```

…then `00 00 00 00` (start marker), exactly `image_size` bytes of JPEG, then
`FF FF FF FF` (end marker). Then it **waits up to 15 s** for a reply.

### 3.2 Depot check-in (fw v7+, no photo)

Same header but `image_size:0` plus `settings_only:1`, and **no image body**.
The Node-RED accumulator branches on that flag to **output 3**, wired straight
to `Fetch Device Command` — bypassing the JPEG write. Without that branch the
accumulator waits forever for an image start marker and the device times out.

### 3.3 Server reply (same socket)

One JSON object, LF-terminated. Either:

```json
{"cmd":"ack"}
{"cmd":"set_config","wake_hour":7,...,"pir_away_mode":1}
{"cmd":"set_sleep","value":900}      // legacy, still supported
```

### 3.4 Device ack

After applying `set_config` **and** saving NVS, the device writes
`CFG_APPLIED\n` on the same socket. Node-RED's accumulator sees it (the
`headerParsed` flag is false at that point) and POSTs
`/api/internal/device-command/<uid>/delivered` with `{"cmd":"set_config"}`.

**Do not** re-introduce optimistic delivery marking. A previous version marked
the command delivered when it was *sent*, so a config the device never received
was recorded as applied. The ack exists specifically to prevent that.

---

## 4. Build / flash / deploy

### Firmware
```bash
cd ~/Documents/PlatformIO/Projects/ESP32-S3-WROOM1-CAM-AI
~/.platformio/penv/bin/pio run                 # build
~/.platformio/penv/bin/pio run -t upload       # flash over USB
~/.platformio/penv/bin/pio device monitor      # 115200, port /dev/cu.usbmodem1301
```

**Always bump `FW_VERSION` in `platformio.ini` when you change firmware.** The
job start prints `[FW] build fw vN (<date> <time>)`. Without a bump you cannot
tell whether a flash actually took — this has already caused three rounds of
debugging a "fix" that was never on the device.

### Platform
```bash
cd ~/AI_BUSINESS_FUN
SKIP_TRAFFIC_AUTOMATION=1 ./deploy-iot-magic.sh pavel_travnicek_89_gmail_com@iot-secure
```
Rsyncs `platform/` → `/opt/iot-dashboard/`, runs `npm install --production` if
needed, restarts the `iot-dashboard` service. Never touches `data/` (production
SQLite + images) or `.env`. Use `DRY_RUN=1` to preview.

The `SKIP_TRAFFIC_AUTOMATION=1` matters: without it the script also ships
`traffic-automation/` and runs `docker compose up -d --build` on the VM. That
tree has uncommitted local changes — only ship it deliberately.

### Node-RED
Import `~/AI_BUSINESS_FUN/NODE-RED/TCP_flows.json` (Menu → Import → **replace
existing nodes**) → Deploy. Timestamped `.bak-*` copies sit alongside the file;
make one before editing.

Flow structure (`Accumulate & correlation header` is the brain, 3 outputs):

```
TCP Server (:8009) → Accumulate & correlation header
                       ├ out0 → Set filename + annotate → Write JPEG to FS → Fetch Device Command
                       ├ out1 → Route by app → Parse telemetry → Influx
                       └ out2 → Fetch Device Command          (settings-only poll)
Fetch Device Command → TCP Reply to Device  (tcp out, mode "reply")
                     → Mark Command Delivered (legacy set_sleep only)
```

Two function nodes declare the `http` module in their **Setup** tab
(`libs: [{var:"http", module:"http"}]`). `require()` does not work in the
function sandbox — use the libs mechanism.

---

## 5. What a healthy run looks like

### Depot / settings-only wake
```
[SLEEP_GEO] early home (43.2m) skip camera/ML — settings poll
[CFG] settings-only poll starting...
[CFG] TCP connected — sending settings-only header
Server response: {"cmd":"set_config",...}
Command: set_config applied win=06:00-22:00 r=500 home=600s away=600s pir(1,1) ...
CFG_APPLIED ack sent
[CFG] settings-only poll complete
[SLEEP_GEO] home -> mode=1 timer=600s pir=0
```

### Normal wake with photo
```
Correlation header sent.
Server response: {"cmd":"ack"}          ← or set_config if one is pending
Classification photo transmitted successfully
[SLEEP_GEO] away -> mode=1 timer=600s pir=0
[PIR] Motion wake disabled for this sleep     ← mode 1 = timer only
```

### Node-RED, same cycle
```
*** Settings-only poll from <uid> — fetching pending command ***
Device command response: {"cmd":"set_config",...}
CFG_APPLIED ack -> delivered for <uid>: {"success":true}
```
and for a photo cycle: `*** JPEG valid. Returning image on output 1 ***` plus
`Tail diagnostics: 4 extra bytes ... (starts with END_MARKER — normal trailer)`.

### Operational-window gate (fw v8)
```
[SCHED] Local time 09:14 (persisted RTC clock, no radio)
[SCHED] Inside window (06:00-22:00) [rtc]. Proceeding.
```
or on a cold boot:
```
[SCHED] No clock yet — registering modem to resolve the hour…
[SCHED] Local time: 09:14 (TZ: modem +8 qh)
[SCHED] Night [modem(cold)] — single 12400s deep sleep until next WAKE
```

---

## 6. Hard-won gotchas (do not re-litigate these)

- **Quectel socket access mode.** `AT+QIRD` only reads from sockets opened with
  `accessMode=0` (buffer). Mode 1 (direct push) delivers the reply as an
  unsolicited URC nobody reads, and every reply-wait times out *silently*. The
  telemetry socket in `sendPhotoOverTCP()` must stay mode 0. Also: the modem
  rejects `AT+QIRD` reads larger than ~640 bytes — keep ≤512.
- **`msg._session` must survive the whole Node-RED flow.** `TCP Reply to Device`
  is a tcp-out in "reply" mode; it writes into the connection identified by
  `msg._session`. Any node that builds a fresh msg must copy it forward or the
  reply is silently dropped.
- **Inter-chunk pacing ≥250 ms in `sendPhotoOverTCP()`.** At 10 ms several TCP
  segments are in flight and, under radio loss, the modem's stack duplicates a
  payload block — the server then receives more bytes than `image_size` and the
  JPEG is corrupt mid-stream. This caused two weeks of missing images.
- **NVS struct changes need a `kSleepGeoMagic` bump**, which resets on-device
  config to defaults on the next boot. Expected — push the desired values right
  after flashing.
- **Devices discovered only from InfluxDB have no SQLite row.** The config PATCH
  handler auto-registers them on first save (the requester has already passed
  the access check). They show up in Admin → Devices named by their UID with no
  client assigned.

---

## 7. Known broken / not proven

1. **OTA is dead.** Every cycle logs `[OTA] no header from server`. The device
   cannot be updated remotely — any firmware fix needs physical access. This is
   the biggest gap before a client deployment. Spec: `HANDOFF-OTA-SERVER-2026-06.md`.
2. **Timezone fallback is UTC.** `FALLBACK_TZ_QUARTER_HOURS = 0`. If the
   operator's network omits the TZ suffix in `CCLK`, the operational window
   shifts 1–2 h in Czechia (CET=4, CEST=8). Unverified in the field — check
   whether real wakes log `TZ: modem` or `TZ: fallback`.
   **RESOLVED 2026-07-29 — the assumption was wrong**: the Czech operator's
   suffix-less CCLK carries **local** time, so fallback 0 was CORRECT. A v9
   "fix" that derived CEST from the UTC date shifted the window −2 h in the
   field; v10 reverted to local-as-is + version-stamped the persisted RTC
   clock. See HIL-RUNBOOK-2026-07.md.
3. **Depot polling costs a full NB-IoT registration every wake** (~30 s GNSS +
   ~90 s modem, measured on a bench cold boot). At `sleep_home_s=600` that is a
   very heavy duty cycle for a parked device. Unknown how much cheaper this gets
   on real PSM deep-sleep wakes where the modem may stay registered — **worth
   measuring in HIL**. `pollSettingsOverTCP()` is called unconditionally.
4. **No sanity check on home coordinates.** A fat-fingered lat/lon silently puts
   the device permanently "away" (observed: home set 134 km from the device).
5. **Pending config re-sends forever** if the ack never lands. Harmless per
   cycle, no retry bound.
6. **The v8 window gate has not been validated on hardware.** It builds clean and
   the logic is reviewed, but the cold-boot and RTC-persisted paths have not been
   exercised on a device. This is a prime HIL target.

---

## 8. Briefing for the HIL test

### 8.1 Knobs you have

Build flags in `platformio.ini`:

- `BENCH_SKIP_UPLINK=1` — skip modem registration/telemetry/OTA entirely; device
  goes GNSS → sleep. Good for isolating power or GNSS behaviour. **Note it also
  skips the depot settings poll** (guarded in `tryEarlyGeofenceSleepRouting`).
- `PIR_WAKE_ENABLE=0` — disable EXT1/PIR wake. Needed when the PIR is physically
  disconnected, because GPIO10 floats and fires wakes within seconds.
- `GNSS_POWER_PERMANENT`, `GNSS_EXTINT_WAKE`, `GNSS_WAKE_PIN=38` — GNSS rail and
  hot-start wake config (PCB reworked 2026-06-19).
- `FW_VERSION` — your flash marker. Bump it every build.

Runtime knobs (via the web app, no reflash):

- **Force home/away**: set `homeLat`/`homeLon` to the bench location (home) or
  somewhere far (away). `homeRadiusM=0` disables the geofence entirely → the
  device always takes the away path.
- **Force night/day**: set `wakeHour`/`napHour` around the current local time.
- **Force PIR behaviour**: `pirHomeMode`/`pirAwayMode` 0/1/2.
- **Shorten cycles**: `sleepHomeS`/`sleepAwayS` = 60 (the minimum) so you are not
  waiting 10 minutes between observations.

### 8.2 What is worth proving

Ranked by risk, given what is already observed working:

1. **v8 window gate, all three time sources.** Cold boot with no clock (expect
   `No clock yet — registering modem`), deep-sleep wake (expect
   `persisted RTC clock, no radio` — this one should use **no** radio, verify
   with current draw or by the absence of registration logs), and the actual
   night decision (set nap a few minutes out, confirm the device sleeps instead
   of running camera+ML).
2. **Depot poll power cost** (open question 3 above). Measure awake time and
   current for a depot wake after a real PSM deep sleep, not a USB cold boot.
   This decides whether the poll needs throttling before a client ships.
3. **Config delivery under adverse radio.** The idempotent-retry design has been
   reasoned about but not stress-tested: kill the connection between the reply
   and the ack, confirm the config stays pending and lands next cycle.
4. **Image integrity at the current 250 ms pacing** over a longer run — the
   corruption bug is fixed but only lightly exercised. Node-RED logs
   `Tail diagnostics` on every finalize; anything other than the 4-byte
   END_MARKER trailer is a regression. Rejected images are dumped to
   `/var/lib/node-red/images/_debug/<mid>.raw` for offline analysis.
5. **Geofence transition** — device moving across the home boundary mid-run, and
   the v7 re-check after a config change moves the home point.

### 8.3 Testing without hardware

The Node-RED accumulator is a pure function and can be unit-tested off-device.
Extract it and run under a mock runtime — this has already caught real bugs:

```bash
python3 -c "
import json
flows=json.load(open('$HOME/AI_BUSINESS_FUN/NODE-RED/TCP_flows.json'))
open('/tmp/acc.js','w').write(next(n['func'] for n in flows if n['id'].startswith('a97faed2')))
"
node -e "
const src=require('fs').readFileSync('/tmp/acc.js','utf8');
const fn=new Function('msg','node','flow','global','context','env','http',src);
const store=new Map();
const flow={get:k=>store.get(k), set:(k,v)=>{v===undefined?store.delete(k):store.set(k,v);}};
const node={warn:console.log, error:console.error};
const http={request:(o,cb)=>({on(){},write(){},end(){}})};
// feed Buffers as msg.payload with a stable msg._session = {id:'t1'}
"
```

Craft a header Buffer, a `00 00 00 00` + JPEG (`FF D8 … FF D9`) + `FF FF FF FF`
body, and assert on the returned outputs. Session state lives in `flow` context
keyed by `_session.id`.

### 8.4 Timing realities

Budget these into any test plan:

- GNSS hot-start fix: **20–35 s** (cold start much longer).
- Modem full registration incl. `AT+NRB` reboot: **~90 s**. A second connection
  in the same wake is cheap (`Module already registered! Minimal setup`).
- Camera warm-up: ~1.6 s for 30 frames.
- Reply-wait: 15 s max (`TCP_RECV_TIMEOUT_MS`).
- Minimum configurable sleep: 60 s.

A single full observation cycle is therefore **~2.5–3 minutes** of awake time.

### 8.5 Rules of engagement

- **Never** deploy with `OVERWRITE_CONFIG=1` unless you mean to replace
  operator-tuned files on the VM.
- Back up `TCP_flows.json` before editing (`.bak-<what>-<HHMMSS>` convention).
- Production telemetry lives in InfluxDB on the VM and SQLite at
  `/opt/iot-dashboard/data/telemetry.db`. The deploy script excludes `data/` —
  keep it that way. To test DB logic locally, copy the file and point
  `IOT_DB_PATH` at the copy.
- The VM root disk was at **94 % (1.8 GB free)** as of 2026-07-02. Check before
  generating large image volumes in a test run.
- Test device UIDs observed in this work: `c6daa2e684d55b02`,
  `649195f647c447b3`. The ENVISAN production units are `ac139cf28f9c2f` and
  `82db05e3e1299312` — **do not** use those as test targets.

---

## 9. Where to look in the code

| What | Where |
|---|---|
| Config struct + NVS | `SleepGeoConfig`, `loadSleepGeoFromNvs`, `saveSleepGeoToNvs` |
| Command handling | `parseAndApplyCommand()` — `set_config` branch |
| Photo upload + reply wait | `sendPhotoOverTCP()` |
| Settings-only poll | `pollSettingsOverTCP()` |
| Geofence routing | `tryEarlyGeofenceSleepRouting()`, `applyDaytimeSleepFromGeofence()` |
| Per-state sleep/PIR | `configureDaytimeSleepForState()` |
| Window gate | `tryScheduleNightSleepIfOutsideWindow()`, `gateTimeFromModem()`, `rtcLocalSecondOfDay()` |
| Server config API | `platform/server.js` — `/api/devices/:uid/config` (GET/PATCH) |
| Internal command API | `platform/server.js` — `/api/internal/device-command/:uid[/delivered]` |
| DB config storage | `platform/lib/db.js` — `setDeviceConfigPending`, `clearDeviceConfigPending`, `deviceConfigToCommand` |
| Web UI | `platform/public/app/screens-container.jsx` — `DeviceSettings` |
