# ESP32-S3-WROOM1-CAM-AI

Firmware for **ESP32-S3** (DFRobot FireBeetle 2 / N16R8) with camera, Edge Impulse inference, and optional Nb-IoT connectivity. Uses the onboard camera to run ML classification (e.g. container level/material) and can report results over TCP or store them locally.

## Hardware

- **Board:** DFRobot FireBeetle 2 ESP32-S3 (or compatible ESP32-S3 with N16R8: 16 MB flash, 8 MB PSRAM)
- **Camera:** DFR0975-style camera (OV5647/OV3660, pinout in code)
- **Sensors:** DFRobot SHT3x (temp/humidity), optional GNSS, SparkFun MAX1704x fuel gauge
- **Display:** U8g2-compatible OLED (I2C)
- **Optional:** Quectel BC95 Nb-IoT modem (UART) for cellular uplink

## Features

- Camera capture and **Edge Impulse** model inference (e.g. container fill level, material type)
- PSRAM-backed frame buffer and EI allocator for large tensors
- Sleep/wake cycles with configurable duration (NVS)
- TCP client to a configurable server (IP/port in code) for sending classification results or photos
- BC95 modem support (AT commands) for Nb-IoT when present
- SHT3x, GNSS, and fuel gauge integration
- Serial debug and optional USB CDC

## Build & upload

1. **Clone the repo**
   ```bash
   git clone https://github.com/PTravnicek/ESP32-S3-WROOM1-CAM-AI.git
   cd ESP32-S3-WROOM1-CAM-AI
   ```

2. **Open in PlatformIO** (VS Code or CLI). Dependencies are in `platformio.ini`; the Edge Impulse deploy is under `lib/deploy`.

3. **Build and upload**
   ```bash
   pio run -t upload
   ```
   Monitor at 115200 baud:
   ```bash
   pio device monitor -b 115200
   ```

## Configuration

- **WiFi:** Not used in the main flow; modem/TCP or local-only use.
- **Server:** `SERVER_IP` and `TCP_PORT` in `src/main.cpp` for the TCP report target.
- **Sleep:** Default sleep duration and modem/camera behaviour are in `src/main.cpp` (e.g. `DEFAULT_SLEEP_TIME`, BC95 pins).
- **Model:** Replace or update the Edge Impulse deploy in `lib/deploy` and the classifier includes in `src/main.cpp` to match your project.

## Repository layout

- `src/main.cpp` – application logic, camera, EI inference, modem, TCP, sleep
- `lib/deploy/` – Edge Impulse C++ library and model
- `platformio.ini` – board, libs, build flags (FireBeetle 2, 16 MB flash, PSRAM, huge partition)
- `include/` – extra headers if needed

## Fork workflow (similar projects)

To start a similar project and still contribute changes back here:

1. **Fork** this repo on GitHub and clone your fork.
2. Develop in the fork; open **Pull Requests** into this repo to bring improvements back.
3. To sync your fork with this repo:
   ```bash
   git remote add upstream https://github.com/PTravnicek/ESP32-S3-WROOM1-CAM-AI.git
   git fetch upstream && git merge upstream/main
   ```

## License

See repository license (if any).
