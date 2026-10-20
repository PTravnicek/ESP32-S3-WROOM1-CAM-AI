# Hand-off: OTA server implementation (June 2026)

What the **server** must implement so the ESP32-S3 AI camera can update its
firmware over NB-IoT. The firmware side is done and compiles clean
(`pio run` → SUCCESS); this document is the spec for whoever builds/operates the
server (whether that is the reference `tools/ota_server.py`, a Node-RED/n8n
flow, or a from-scratch service).

Authoritative source: `runOtaUpdate()` in `src/main.cpp`. Reference
implementation: `tools/ota_server.py`. Background/rationale: `OTA-NB-IoT.md`.

---

## 0. TL;DR for the server developer

Build a **plain TCP server** that:

1. Listens on a TCP port (default **8010**) reachable from the cellular network.
2. Reads **one line** from the device: `OTA_CHECK device_id=<hex> ver=<int>\n`.
3. Decides if a newer firmware exists for that device.
4. Replies either:
   - `OTA_NONE\n` (and close), or
   - `OTA_BEGIN size=<bytes> ver=<int> md5=<hex>\n` immediately followed by
     **exactly `<bytes>` raw bytes** of the firmware `.bin`, then close.
5. Closes the connection. The device does the rest (verify + flash + reboot).

That's the whole contract. Everything below is detail and operational guidance.

---

## 1. Network / transport

- **Protocol:** raw TCP (no HTTP, no TLS, no framing library). The device uses
  the Quectel modem's `AT+QIOPEN`/`AT+QIRD` TCP stack in *buffer access mode*.
- **Default endpoint:** `34.10.203.180:8010` (compiled into firmware as
  `OTA_SERVER_IP` / `OTA_TCP_PORT`; OTA runs on a **different port** from
  telemetry, which stays on **8009**).
- **One connection = one update transaction.** The device opens, checks,
  optionally downloads, then closes. There is no keep-alive, no pipelining.
- **Firewall:** the chosen port must be open inbound to the VM. On GCP add an
  ingress rule for `tcp:8010`. Prefer restricting the source to the mobile
  operator's NB-IoT IP range over `0.0.0.0/0` if you can obtain it.

---

## 2. Wire protocol (exact)

### 2.1 Request (device → server)

A single CRLF-or-LF-terminated ASCII line, e.g.:

```
OTA_CHECK device_id=1a2b3c4d5e6f7080 ver=1\n
```

- `device_id` — the device's 64-bit id in **lowercase hex, no `0x`**, no
  zero-padding guarantee (it is `String(device_id, HEX)`; treat it as an opaque
  hex token). Use it for per-device targeting/audit; it is **not** required to
  decide whether to update.
- `ver` — the **integer** firmware version currently running on the device
  (`FW_VERSION`). This is the value you compare against.
- The server should read until the first `\n`. Be tolerant: parse `device_id=`
  and `ver=` with a regex/substring scan; ignore unknown tokens and surrounding
  whitespace. If the line does not start with `OTA_CHECK`, reply `OTA_NONE\n`.

### 2.2 Response — no update

```
OTA_NONE\n
```

Send this (and close) when the latest available version is **≤** the device's
reported `ver`, or when you have no image to serve. The device logs "up to date"
and continues normal operation.

### 2.3 Response — update available

A single header line, then the raw binary, then close:

```
OTA_BEGIN size=<bytes> ver=<int> md5=<hex>\n
<...exactly <bytes> raw bytes of firmware.bin...>
```

- `size` — **decimal byte count** of the `.bin`. The device reads **exactly**
  this many bytes; do not send more or fewer. Must be `> 0` and must fit the OTA
  slot (≤ **5 MB**; current images are ~2.3 MB).
- `ver` — integer version of the image you are sending (for logging; the device
  stores nothing but logs `v<old> -> v<new>`).
- `md5` — **32-char lowercase hex** MD5 of the **entire `.bin`**. The device
  recomputes MD5 over the received bytes and **aborts on mismatch**. (If you
  send a malformed/empty `md5`, the device skips the MD5 check but the ESP32
  bootloader still validates the image; always send a correct MD5.)
- After the last byte, **close the socket**. A brief (~1 s) pause before close is
  good practice so the modem can drain its receive buffer.

### 2.4 Byte-order / encoding notes

- The header line is ASCII. Token order shown is what the reference server
  emits; the device parses by key (`size=`, `ver=`, `md5=`) so order is not
  strictly required, but keep `OTA_BEGIN` as the first token of the line.
- The body is the **firmware.bin verbatim** — the same file produced by the
  build at `.pio/build/dfrobot_firebeetle2_esp32s3/firmware.bin`. Do **not**
  base64/hex-encode it, do not gzip it, do not add length prefixes per chunk.
  You may `send()` it in any chunk sizes you like (TCP reassembles it); the
  device pulls it back in modem-sized batches.

---

## 3. Version & targeting logic the server owns

The device is intentionally dumb: it just reports its version and trusts the
server. The server decides everything else.

- **Minimum viable:** keep a single "latest" image with an integer version. Send
  it to any device whose `ver` is lower; otherwise `OTA_NONE`.
- **Reference behavior (`ota_server.py`):** the latest image is the highest
  `firmware_v<NN>.bin` in `--firmware-dir`. The version is parsed from the
  filename, so publishing = dropping `firmware_v<NN>.bin` in that folder.
  **The filename version must match the compiled `FW_VERSION`** of that build.
- **Recommended additions for a fleet (not in the reference server):**
  - **Staged rollout / canary:** maintain a `device_id → allowed_version` map
    so you can update one device first, then widen. Default unknown devices to
    "no update" or to a stable channel.
  - **Pinning / rollback:** allow targeting a specific (possibly lower) version
    for a given `device_id` to recover a misbehaving unit (the device flashes
    whatever you send as long as it's a valid image; "newer" is your policy, not
    the device's).
  - **Audit log:** record `device_id`, reported `ver`, served `ver`, bytes sent,
    duration, outcome. The reference server prints these to stdout/journal.

---

## 4. Operational requirements

- **Throughput is tiny.** The modem UART is 9600 baud (~960 B/s), so a ~2.3 MB
  image takes ~35–40 min per device on the wire. The server must:
  - Tolerate **long-lived connections** (tens of minutes). Set generous socket
    timeouts; do not assume a stalled-looking but slow transfer is dead.
  - Handle **concurrent devices** (thread/async per connection). The reference
    server is threaded.
  - Optionally **pace** sends (`--pace-ms`) if a particular modem/firmware
    combination overruns; default 0 is fine for current firmware.
- **Idempotency / retries.** A device may disconnect mid-download (timeout, lost
  coverage, power) and retry on its next trigger. Each connection is
  independent and must restart the transfer from byte 0. No server-side resume
  is needed (the device does not request ranges).
- **Robustness:** never send a partial/corrupt image deliberately; on any
  server-side read/file error before sending the header, just send `OTA_NONE\n`.
- **Memory:** reading the whole `.bin` into RAM per request is fine (~2–3 MB);
  for many concurrent devices prefer streaming from disk.

---

## 5. Security considerations (important — currently NONE)

The protocol as shipped is **unauthenticated and unencrypted**. Anyone who can
reach the port can be served firmware, and the link is plaintext. Before any
real deployment, the server developer should add at least:

1. **Source restriction** — firewall the port to the operator's NB-IoT egress
   range; do not expose `0.0.0.0/0` long-term.
2. **Device allow-list** — only serve images to known `device_id`s.
3. **Signed images (recommended)** — enable ESP32 **Secure Boot v2** /
   signed-app verification on the firmware side so the device only boots
   correctly signed images, making a rogue OTA server unable to push runnable
   code. (Firmware change; coordinate with the firmware owner.)
4. **Transport** — NB-IoT TCP to a known operator APN limits exposure, but it is
   still plaintext. If threat model requires it, move to the modem's TLS socket
   (`AT+QSSLOPEN`) — this is a firmware + server change and is **not** in scope
   of the current implementation.

For bench/PoC on a private APN, items 1–2 are the practical minimum.

---

## 6. Deployment (reference server on the VM)

The same GCP VM that ingests telemetry can host OTA (different port). Full steps
with the `systemd` unit are in `OTA-NB-IoT.md §ServerSide / Option A`. Short
version:

```bash
scp tools/ota_server.py user@34.10.203.180:/opt/ota/ota_server.py
ssh user@34.10.203.180 'sudo mkdir -p /opt/ota/firmware'
# open tcp:8010 in GCP firewall + host firewall
# install /etc/systemd/system/ota.service (see OTA-NB-IoT.md), then:
sudo systemctl enable --now ota.service
sudo journalctl -u ota.service -f
```

**Publishing an update:** bump `FW_VERSION` in `platformio.ini`, `pio run`, then
copy `firmware.bin` to the VM as `firmware_v<NN>.bin` with `<NN>` matching the
new `FW_VERSION`. No restart needed.

---

## 7. Node-RED / n8n integration (control plane only)

Neither tool is a good raw-TCP byte pump for a multi-MB blob. Keep
`ota_server.py` as the endpoint devices connect to, and use Node-RED/n8n to:

- **Publish images** — on a release webhook / dashboard button, SFTP/SSH the new
  `firmware_v<NN>.bin` into `/opt/ota/firmware/`.
- **Observe** — tail the server log / journal onto a dashboard.
- **Command-trigger updates** — if you enable the telemetry reply path
  (`TCP_WAIT_FOR_SERVER_RESPONSE 1` in firmware), your **telemetry** server/flow
  can reply to a chosen `device_id` with `{"cmd":"ota"}`; the device then runs
  the OTA check at the end of that cycle. This reply is part of the **telemetry**
  protocol (port 8009), not the OTA protocol — see `parseAndApplyCommand()`.

If you insist on pure Node-RED for the TCP transfer, use a `tcp in` node in
"stream of buffers / single connection" mode, buffer the first line, emit the
`OTA_BEGIN ...\n` header buffer then the file buffer to `tcp out`. You must
handle Buffer concatenation and back-pressure yourself — `ota_server.py` already
does this correctly, so this is not recommended.

---

## 8. Acceptance test (server side)

You can validate the server without a device using `netcat`/`socat`:

```bash
# expect: OTA_NONE  (latest published <= 99)
printf 'OTA_CHECK device_id=deadbeef ver=99\n' | nc <host> 8010

# expect: OTA_BEGIN size=... ver=... md5=...  then binary  (device on v0)
printf 'OTA_CHECK device_id=deadbeef ver=0\n' | nc <host> 8010 > /tmp/got.bin
# strip the header line, then compare md5 of the body to the advertised md5
```

Real end-to-end: power-cycle a device (cold-boot check). Serial shows
`[OTA] check start ...` → progress `[OTA] N/M (xx%)` → reboot into the new
version; server log shows the device's `device_id`, reported version, and bytes
sent.

---

## 9. Open items / decisions for the server owner

1. **Auth & signing** (§5) — required before production; currently none.
2. **Rollout policy** — single "latest" vs per-device channels/canary (§3).
3. **Faster transfers** — if 35–40 min/device is unacceptable, the firmware can
   raise the modem UART to 115200 (≈12×); coordinate with firmware owner
   (`OTA-NB-IoT.md §Making it faster`). No server change needed.
4. **Trigger model** — cold-boot check (default, good for bench) vs
   server-command after telemetry (production). Affects whether you also touch
   the telemetry flow.
```
