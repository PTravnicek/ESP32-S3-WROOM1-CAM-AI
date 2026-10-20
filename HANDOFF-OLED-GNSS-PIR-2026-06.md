# Hand-off: OLED, GNSS power, PIR scheduling (June 2026)

Session summary of firmware changes to `src/main.cpp` (DFRobot FireBeetle 2 ESP32-S3,
N16R8). All changes compile clean (`pio run` → SUCCESS). Board: camera **and GNSS** on
`LD_SWCH_1`, OLED + fuel gauge on `LD_SWCH_2`, PIR on **GPIO10**.
(Note: an earlier "GNSS on permanent 3.3 V" assumption was corrected — GNSS is switched.)

---

## 1. Camera — 0 frames after any non-power-on reset (ROOT CAUSE, fixed)
- **Symptom:** camera streamed 30/30 only after a true power-on (cold boot). After **every**
  deep-sleep wake *and* every warm reset (`esp_restart`, USB-CDC reconnect) it initialized
  fine (SCCB `0x30` ACK) but warmed up **0/30 frames**.
- **Diagnosis path (what it was NOT):** not the camera HW (same after swap), not rail
  discharge (VDD measured dropping to 0 V), not RESET wiring (GPIO38 continuity verified),
  not RTC-pin isolation, not LCD_CAM/GDMA state. The decisive test was a compile-time flag
  that does `esp_restart()` instead of deep sleep: a plain reboot reproduced the failure,
  proving it was **SoC state surviving a non-POR reset**, not deep sleep or hardware.
- **Root cause:** `enterSleepMode()` latches every camera DVP pin LOW before sleep via
  `holdPinLowForSleep()` → `gpio_hold_en` / `rtc_gpio_hold_en` on
  `SDA, SCL, XCLK, PCLK, VSYNC, HREF, Y2..Y9`. These per-pin hold latches live in the
  RTC/AON domain and **survive any reset except a true power-on**. But `setup()` only
  released **5** of them (`LD_SWCH_1/2, SDA, SCL, XCLK`). So `PCLK/VSYNC/HREF/Y2..Y9`
  stayed clamped LOW, the ESP held the sensor's data/sync outputs down → 0 frames. Cold
  boot worked only because POR clears the RTC/AON latches.
- **Fix:** expanded `heldPins[]` at the top of `setup()` to **mirror the hold set exactly**
  (adds `PCLK, VSYNC, HREF, Y2..Y9`). The release loop runs unconditionally, so it fixes
  both deep-sleep wake and warm-reset paths. Verified 30/30 on warm-reset cycles.
- **Hardening kept:** `periph_module_reset(PERIPH_LCD_CAM_MODULE)` and `PERIPH_GDMA_MODULE`
  before camera init (makes warm resets / USB-debug also stream cleanly); graceful
  "camera unavailable -> sleep and retry" instead of a 2-min spin; OLED per-attempt status.

### Camera RESET decision (final): power-line, no PCB change
- The camera RESET stays on the **`LD_SWCH_1` rail (RC reset)** — `RESET_GPIO_NUM = -1`.
  The rail-cycle reset was always sufficient; the GPIO38 reset line did **not** contribute
  to the fix and is not needed. **No PCB modification required.**
- The diagnostic GPIO38 wiring + explicit reset pulse were removed (`RESET_GPIO_NUM = -1`
  compiles the pulse out). If reusing the bench unit that was rewired to GPIO38, move its
  RESET wire back to the `LD_SWCH_1` rail so it matches production.
- The `esp_restart` diagnostic flag `CAM_DEBUG_RESTART_INSTEAD_OF_SLEEP` is back to `0`
  (real deep sleep); keep it for future warm-vs-POR isolation if needed.

### Earlier camera note (superseded)
- The first camera-vs-PIR SCCB failures were a separate, genuinely faulty camera (HW swap).
  The 0-frames-after-wake issue above is unrelated and was a firmware regression.

## 2. OLED boot / loading screen
- `oled_draw_splash_logo()` — logo extracted for reuse.
- `oled_boot_loading(ms)` — animated boot screen (bouncing bar) so the device clearly
  shows it is alive.
- `oled_bring_up_early()` — brings the OLED up **before** the long pre-camera GNSS wait
  (inside `tryEarlyGeofenceSleepRouting`) instead of only after camera init (~t=67 s).
  Constraint: OLED shares the camera I2C bus (GPIO1/2), so it still can't come up until
  the bus is initialized.

## 3. GNSS "searching" animation
- `oled_gnss_search_frame()` — full-screen animation (globe + orbiting satellite +
  pulsing ring + caption + countdown bar).
- Driven via a new optional `GnssProgressCb` parameter on `gnssReadLatestFixSinceWake()`;
  `gnssSearchAnimTick()` throttles to ~11 fps and restores the I2C clock after each draw.

## 4. GNSS faster fix (hot start)
- Module is on permanent 3.3 V → keeps tracking across deep sleep.
- `gnssReadLatestFixSinceWake()` rewritten: tight stale-buffer drain (`GNSS_DRAIN_MAX_MS`),
  returns on **first** valid fix (RMC *or* GGA) with a short grace window
  (`GNSS_META_GRACE_MS`) for sat count; logs `[GNSS] fix in Nms (... hot-start)`.

## 5. GNSS low-power during sleep (backup mode)
- `gnssSendUbx()` (checksum computed in code), `gnssEnterBackup()` (UBX-RXM-PMREQ backup),
  `gnssWakeFromBackup()` (I2C nudge on wake; read retries once on no-ACK).
- `enterSleepMode()` sends backup as the **last** I2C action; SDA/SCL driven low **SCL
  first** to avoid an I2C START edge that would wake the module.
- **Caveat:** this was built when GNSS was believed to be on permanent 3.3 V. GNSS is
  actually on `LD_SWCH_1`, so it powers off during deep sleep regardless of the backup
  command — the PMREQ/backup is mostly redundant for power (rail cut dominates) and
  contributes to the slower warm-start fix times (~20–40 s vs sub-second hot start).
  Reconsider whether to keep the backup command or simplify, once the GNSS wiring is final.

## 6. PIR false wakes (diagnosis only — hardware)
- Confirmed by user tests (aluminium box still woke it; disconnecting sensor stopped wakes):
  the **HC-SR501 at 3.3 V is out of spec (4.5–20 V)** and free-runs, driving GPIO10 HIGH.
  Firmware already verifies OUT LOW before arming and requires sustained HIGH on wake, so
  it is not a firmware pull-high.
- Fix is hardware: power HC-SR501 at **5 V** with a divider on OUT (e.g. 10k/15k → ~3.0 V,
  ESP is **not** 5 V tolerant, max ~3.6 V), common ground, short cable — **or** swap to an
  **AM312** (native 3.3 V). No firmware change.

## 7. PIR-driven wake scheduling (behavior change)
- PIR is now the **primary daytime wake source**; the 600 s / 1800 s intervals
  (`sleep_away_s` / `sleep_home_s`) are no longer used for timing (kept in NVS/remote-config
  for compatibility).
- New `secondsUntilNextNap()` + `daytimeFallbackSleepSeconds()`; gate records
  `g_localSecOfDay`. Daytime sleeps now target the **NAP boundary** (timer only bridges
  day→night); `applyDaytimeSleepFromGeofence()` and the early-geofence "home" path updated.
- Night unchanged: PIR disabled, timer sleeps until WAKE (morning), then location check.
- Optional, not implemented: `DAYTIME_MAX_SLEEP_S` safety cap if the PIR fails.

## 8. OLED time/signal blank during telemetry (fix)
- On cold boot the modem is freshly (re)registered, so the single `AT+CSQ` / `AT+CCLK`
  reads before TX returned `99` / no-time → blank bars and `--:--`.
- `updateOledSignalFromCSQ()` now retries up to 5× (600 ms) until a valid RSSI.
- OLED clock is **seeded at the schedule gate** (early reliable `[SCHED] Local time`), so
  the top bar is never blank; later query still refreshes it.

---

## Key tuning constants (`src/main.cpp`)
| Define | Purpose |
|--------|---------|
| `GNSS_DRAIN_MAX_MS` (250) | max stale-buffer drain after wake |
| `GNSS_META_GRACE_MS` (1500) | wait for GGA sats after first fix |
| `WAKE_HOUR` (7) / `NAP_HOUR` (20) | daytime operational window |
| `PIR_WAKE_CONFIRM_MS` (120) | sustained-HIGH needed to count a wake as motion |
| `CAM_DEBUG_RESTART_INSTEAD_OF_SLEEP` (0) | diag: reboot instead of deep sleep (warm-vs-POR isolation) |
| `RESET_GPIO_NUM` (-1) | camera RESET is on the LD_SWCH_1 rail (RC) — no dedicated GPIO |

## Resolved this session
- **Camera 0-frames after wake** — fixed (mirror hold/release set; see §1). Verified 30/30.
- **Camera RESET** — stays on the power-line rail; no PCB change.

## Suggested next steps
1. Fix PIR power (5 V + divider, or AM312) — prerequisite for testing #7 cleanly.
2. Measure deep-sleep current; decide whether to keep the GNSS PMREQ/backup (§5 caveat).
3. Decide on optional `DAYTIME_MAX_SLEEP_S` safety cap.
4. If the bench unit's RESET was rewired to GPIO38, move it back to the `LD_SWCH_1` rail.
