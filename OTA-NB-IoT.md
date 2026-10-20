# OTA firmware updates over NB-IoT

This document describes how the ESP32-S3 AI camera pulls new firmware over the
Quectel NB-IoT modem, and the exact steps to set up the server side (plain VM,
Node-RED, or n8n).

## How it works (firmware side)

The device reuses the **same TCP/IP AT-command stack** it already uses to upload
photos (`AT+QIOPEN` / `AT+QISEND` / `AT+QIRD` / `AT+QICLOSE`). No HTTP, TLS, or
modem-FOTA needed.

```
Device  --(TCP, port 8010)-->  OTA server
   |  OTA_CHECK device_id=<hex> ver=<int>\n
   |
   |  <-- OTA_NONE\n                                   (already up to date)
   |  -- or --
   |  <-- OTA_BEGIN size=<bytes> ver=<int> md5=<hex>\n
   |  <-- <raw firmware.bin bytes>
   |
   v  esp_ota_write() into the spare slot -> verify MD5 -> set boot -> reboot
```

Safety:

- The new image is written to the **inactive** OTA slot. The running firmware is
  never overwritten.
- The download is **MD5-verified end-to-end**, and the ESP32 bootloader
  validates the image header/checksum on `esp_ota_end()`. A truncated, corrupt,
  or interrupted download is rejected and the device simply keeps running the
  current firmware.
- Only after both checks pass does it flip the boot partition and reboot.

Relevant code: `runOtaUpdate()` in `src/main.cpp`. Config defines (top of
`main.cpp`, overridable via `build_flags`):

| Define | Default | Meaning |
|--------|---------|---------|
| `FW_VERSION` | `1` (set in `platformio.ini`) | Version this build reports. **Bump on every release.** |
| `OTA_SERVER_IP` | `SERVER_IP` (`34.10.203.180`) | OTA server host |
| `OTA_TCP_PORT` | `8010` | OTA server port |
| `OTA_CHECK_ON_COLD_BOOT` | `1` | Check on every true power-on (great for bench testing) |
| `OTA_VIA_SERVER_COMMAND` | `1` | Honor a server `{"cmd":"ota"}` reply after a telemetry upload |

### When the device checks for updates

1. **Cold boot** (`OTA_CHECK_ON_COLD_BOOT=1`): on every true power-on. Best for
   bench testing — just power-cycle the board.
2. **Server command** (`OTA_VIA_SERVER_COMMAND=1`): after a successful telemetry
   upload, if the server's reply is `{"cmd":"ota"}`. This requires the telemetry
   reply path to be enabled — set `#define TCP_WAIT_FOR_SERVER_RESPONSE 1` in
   `main.cpp` (it is `0` by default). This is the recommended **production**
   trigger: no extra airtime unless you actually publish an update.

> NB-IoT is slow. The modem UART in this firmware runs at **9600 baud**
> (`bc95_modem.begin(9600, ...)`), i.e. ~960 B/s max, so a ~1.5 MB image takes
> on the order of 25–40 minutes. See "Making it faster" at the end.

---

## One-time: switch to an OTA-capable partition table

`huge_app.csv` has a **single** app slot and cannot do OTA. This repo now ships
`partitions_ota_16mb.csv` with two 5 MB slots, already wired into
`platformio.ini`. The `nvs`/`otadata` offsets match `huge_app.csv`, so the
stored `device_id` survives the switch.

**This first flash must be done over USB** (you can't change the partition
layout over the air). After this, all future updates can be OTA.

```bash
# build + flash the OTA-capable firmware once over USB
pio run -t upload
```

Verify in the boot log that you now have two app partitions and OTA data.

---

## Server side

You need a TCP server reachable at `OTA_SERVER_IP:OTA_TCP_PORT` (default
`34.10.203.180:8010`) that speaks the protocol above. This repo ships a
dependency-free reference implementation: `tools/ota_server.py`.

It serves the highest-versioned `firmware_v<NN>.bin` from a folder. A device
reporting a lower `ver=` gets that image; otherwise it gets `OTA_NONE`.

### Option A — Plain VM (recommended, simplest)

1. **Copy the server to the VM** (the same GCP VM at `34.10.203.180` that
   receives telemetry is fine — OTA uses a different port, 8010):

   ```bash
   scp tools/ota_server.py user@34.10.203.180:/opt/ota/ota_server.py
   ssh user@34.10.203.180
   sudo mkdir -p /opt/ota/firmware
   ```

2. **Open the firewall port** so the modem can reach it.
   - On the VM: `sudo ufw allow 8010/tcp` (if ufw is used).
   - On GCP: add an ingress firewall rule for **TCP 8010** to the VM
     (VPC network → Firewall → Create rule → tcp:8010, source `0.0.0.0/0` or,
     better, your mobile operator's NB-IoT IP range).

3. **Publish a firmware image.** Build it, then name it by version and drop it
   in the firmware folder:

   ```bash
   # on your dev machine
   pio run                       # produces .pio/build/dfrobot_firebeetle2_esp32s3/firmware.bin
   # bump FW_VERSION in platformio.ini BEFORE building the image you publish,
   # e.g. -DFW_VERSION=2, so devices still on v1 see it as newer.
   scp .pio/build/dfrobot_firebeetle2_esp32s3/firmware.bin \
       user@34.10.203.180:/opt/ota/firmware/firmware_v2.bin
   ```

   > The version in the **filename** (`firmware_v2.bin`) is what the server
   > advertises; make it match the `FW_VERSION` you compiled in.

4. **Run it as a service** so it survives reboots. Create
   `/etc/systemd/system/ota.service`:

   ```ini
   [Unit]
   Description=NB-IoT OTA server
   After=network.target

   [Service]
   ExecStart=/usr/bin/python3 /opt/ota/ota_server.py --firmware-dir /opt/ota/firmware --port 8010
   Restart=always
   User=www-data

   [Install]
   WantedBy=multi-user.target
   ```

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now ota.service
   sudo journalctl -u ota.service -f       # watch device checks live
   ```

5. **Test.** Power-cycle the device (cold-boot check). In the serial monitor you
   should see `[OTA] check start ...` then either `up to date` or a download
   progress (`[OTA] N/M (xx%)`) ending in a reboot into the new version. The
   server log shows the device's `device_id`, reported version, and bytes sent.

**To push a new version later:** bump `FW_VERSION`, build, copy the `.bin` as
`firmware_v<NN>.bin` into `/opt/ota/firmware/`. Done — no service restart needed.

### Option B — Node-RED

Node-RED's default TCP nodes are line/stream oriented and awkward for a
length-prefixed binary blob. Two clean ways to use Node-RED:

- **Recommended:** keep `tools/ota_server.py` running (Option A) as the byte
  pump, and use Node-RED only as the **control plane / dashboard**:
  - An `exec` node (or `file` node) to drop a new `firmware_v<NN>.bin` into
    `/opt/ota/firmware/` when you click a dashboard button or receive an MQTT
    message.
  - A `file in` / `tail` node on `journalctl` (or have the Python server append
    to a log file) to show OTA progress on a Node-RED dashboard.
  - If you drive updates by the **server-command** path, your existing telemetry
    flow can reply to a device with `{"cmd":"ota"}` for selected `device_id`s
    (e.g. a Node-RED `switch` on device id → `function` returns that JSON).

- **Pure Node-RED TCP** (only if you want to drop the Python server): use a
  `tcp in` node in **"single connection / stream of buffers"** mode on port
  8010, a `function` node that buffers the first line, looks up the latest
  firmware, and emits the `OTA_BEGIN ...\n` header buffer followed by the file
  contents buffer (read with a `file in` node), into a `tcp out` node tied to
  the same connection. This works but you must handle Buffer concatenation and
  back-pressure yourself; the Python server already does this correctly, so
  Option A is less effort.

### Option C — n8n

n8n is HTTP/workflow oriented and **cannot easily serve a raw TCP byte stream**,
so don't make n8n the OTA byte pump. Use it as the **release orchestrator**:

- A workflow triggered by a GitHub release / webhook / manual button that:
  1. downloads/receives the new `firmware.bin`,
  2. uses an **SSH** or **SFTP** node to copy it to the VM as
     `/opt/ota/firmware/firmware_v<NN>.bin`,
  3. optionally posts a Slack/Telegram message "firmware vNN published".
- Keep `tools/ota_server.py` (Option A) as the thing devices actually connect to.
- If you use the server-command trigger, n8n can also call your telemetry
  backend to mark certain `device_id`s as "update on next upload".

**Summary:** `ota_server.py` on the VM is the actual OTA endpoint. Node-RED/n8n
are best used to *publish images* and *trigger/observe* updates, not to stream
the bytes.

---

## Rollback (optional, recommended for fleet)

The build-in safety already prevents bricking from a bad download. For extra
protection against a firmware that boots but is broken, enable ESP-IDF
**app rollback**:

1. Enable `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` (sdkconfig / menuconfig).
2. After a new image proves itself (e.g. one full successful wake→upload cycle),
   call `esp_ota_mark_app_valid_cancel_rollback()`. If it crashes/reboots before
   marking valid, the bootloader automatically reverts to the previous slot.

This is left out of the default build to avoid changing the Arduino sdkconfig;
add it when you move from bench testing to a deployed fleet.

---

## Making it faster (NB-IoT throughput)

The dominant bottleneck is the **9600-baud modem UART**, not the air interface.
Options, in order of impact:

1. **Raise the modem UART baud.** Configure the Quectel module for 115200
   (`AT+IPR=115200;&W`) and change `bc95_modem.begin(9600, ...)` to `115200`.
   That alone gives ~12× faster downloads. Validate your wiring handles it.
2. **Send only deltas** (ship smaller images / strip debug). `CORE_DEBUG_LEVEL`
   is currently `5`; lowering it shrinks the binary.
3. **Keep the device awake and powered** during the download (USB or good
   battery) — the cycle safety-timeout will abort a long download, so for large
   images consider raising the timeout used by `isCycleTimeoutExceeded()` during
   OTA, or run OTA on the cold-boot path where you control timing.
