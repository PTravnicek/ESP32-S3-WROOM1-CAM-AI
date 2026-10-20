#!/usr/bin/env python3
"""
Minimal NB-IoT OTA server for the ESP32-S3 AI camera firmware.

Protocol (matches runOtaUpdate() in src/main.cpp):

  Device -> Server (one line):
      OTA_CHECK device_id=<hex> ver=<int>\n

  Server -> Device:
      OTA_NONE\n                                   (device already up to date)
    or
      OTA_BEGIN size=<bytes> ver=<int> md5=<hex>\n  followed by exactly <bytes>
                                                    of the raw firmware .bin

The newest firmware is whatever .bin sits in --firmware-dir with the highest
version. Version is taken from the filename: firmware_v<NN>.bin (e.g.
firmware_v7.bin -> version 7). Drop a new firmware_v<NN>.bin in that folder and
devices reporting a lower version will pull it on their next check.

Run:
    python3 ota_server.py --firmware-dir ./firmware --port 8010

This is intentionally dependency-free (stdlib only) so it runs anywhere.
"""

import argparse
import hashlib
import os
import re
import socket
import socketserver
import threading
import time

FW_RE = re.compile(r"firmware_v(\d+)\.bin$", re.IGNORECASE)


def find_latest_firmware(firmware_dir):
    """Return (version:int, path:str) of the highest-versioned firmware, or (0, None)."""
    best_ver, best_path = 0, None
    if not os.path.isdir(firmware_dir):
        return best_ver, best_path
    for name in os.listdir(firmware_dir):
        m = FW_RE.search(name)
        if not m:
            continue
        ver = int(m.group(1))
        if ver > best_ver:
            best_ver, best_path = ver, os.path.join(firmware_dir, name)
    return best_ver, best_path


def read_request_line(conn, timeout=30):
    """Read a single '\n'-terminated line from the socket."""
    conn.settimeout(timeout)
    buf = bytearray()
    while b"\n" not in buf:
        chunk = conn.recv(256)
        if not chunk:
            break
        buf.extend(chunk)
        if len(buf) > 512:
            break
    return buf.split(b"\n", 1)[0].decode(errors="replace").strip()


class OtaHandler(socketserver.BaseRequestHandler):
    firmware_dir = "./firmware"
    send_chunk = 1024           # bytes per send() (NB-IoT links are tiny)
    pace_ms = 0                 # optional inter-chunk delay to avoid modem overrun

    def handle(self):
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        try:
            line = read_request_line(self.request)
        except socket.timeout:
            print(f"[{peer}] timeout waiting for request")
            return
        print(f"[{peer}] request: {line!r}")

        if not line.startswith("OTA_CHECK"):
            self.request.sendall(b"OTA_NONE\n")
            return

        dev_ver = 0
        m = re.search(r"ver=(\d+)", line)
        if m:
            dev_ver = int(m.group(1))
        dev_id = ""
        m = re.search(r"device_id=([0-9a-fA-F]+)", line)
        if m:
            dev_id = m.group(1)

        latest_ver, path = find_latest_firmware(self.firmware_dir)
        if not path or latest_ver <= dev_ver:
            print(f"[{peer}] dev={dev_id} v{dev_ver}, latest=v{latest_ver} -> OTA_NONE")
            self.request.sendall(b"OTA_NONE\n")
            return

        with open(path, "rb") as f:
            data = f.read()
        md5 = hashlib.md5(data).hexdigest()
        header = f"OTA_BEGIN size={len(data)} ver={latest_ver} md5={md5}\n"
        print(f"[{peer}] dev={dev_id} v{dev_ver} -> sending {os.path.basename(path)} "
              f"({len(data)} bytes, v{latest_ver}, md5={md5})")
        self.request.sendall(header.encode())

        sent = 0
        t0 = time.time()
        while sent < len(data):
            chunk = data[sent:sent + self.send_chunk]
            self.request.sendall(chunk)
            sent += len(chunk)
            if self.pace_ms:
                time.sleep(self.pace_ms / 1000.0)
        dt = time.time() - t0
        rate = (sent / 1024.0) / dt if dt > 0 else 0
        print(f"[{peer}] sent {sent} bytes in {dt:.1f}s ({rate:.1f} kB/s)")
        # Give the device a moment to drain before the socket closes.
        time.sleep(1.0)


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser(description="NB-IoT OTA server for ESP32-S3 AI camera")
    ap.add_argument("--firmware-dir", default="./firmware",
                    help="folder holding firmware_v<NN>.bin files")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8010)
    ap.add_argument("--chunk", type=int, default=1024, help="bytes per send()")
    ap.add_argument("--pace-ms", type=int, default=0,
                    help="delay between chunks (ms); raise if the modem overruns")
    args = ap.parse_args()

    OtaHandler.firmware_dir = args.firmware_dir
    OtaHandler.send_chunk = args.chunk
    OtaHandler.pace_ms = args.pace_ms

    ver, path = find_latest_firmware(args.firmware_dir)
    print(f"OTA server on {args.host}:{args.port}")
    print(f"firmware dir: {os.path.abspath(args.firmware_dir)}")
    print(f"latest firmware: {'v%d (%s)' % (ver, os.path.basename(path)) if path else 'NONE'}")

    with ThreadedTCPServer((args.host, args.port), OtaHandler) as srv:
        srv.serve_forever()


if __name__ == "__main__":
    main()
