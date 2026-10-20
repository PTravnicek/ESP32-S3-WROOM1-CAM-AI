#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sys/time.h>
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "mbedtls/sha256.h"
#include "esp_rom_md5.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <WiFi.h>
#include <cmath>
#include <ArduinoJson.h>

#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "model-parameters/model_variables.h"

// Load switches (active high, keep ACTIVE for peripherals)
#define LD_SWCH_1 13  // Camera + GNSS (jumper VDD1). Camera only if GNSS on 3V1 jumper.
#define LD_SWCH_2 14  // OLED, MAX17048

// Camera pins (DFR0975)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1  // RESET is on the LD_SWCH_1 rail (RC reset) — no dedicated GPIO.
                             // The real wake bug was firmware (held DVP pins not released),
                             // now fixed, so rail-cycle reset is sufficient. No PCB change.
#define SIOD_GPIO_NUM    1
#define SIOC_GPIO_NUM    2
#define VSYNC_GPIO_NUM   6 // OK CONNECTION ON BOARD
#define HREF_GPIO_NUM    42 // OK CONNECTION ON BOARD
#define XCLK_GPIO_NUM    45 // OK CONNECTION ON BOARD
#define PCLK_GPIO_NUM    5 // OK CONNECTION ON BOARD
#define Y9_GPIO_NUM      48 // OK CONNECTION ON BOARD
#define Y8_GPIO_NUM      46 // OK CONNECTION ON BOARD
#define Y7_GPIO_NUM      8 // OK CONNECTION ON BOARD
#define Y6_GPIO_NUM      7 // OK CONNECTION ON BOARD
#define Y5_GPIO_NUM      4 // OK CONNECTION ON BOARD
#define Y4_GPIO_NUM      41 // OK CONNECTION ON BOARD
#define Y3_GPIO_NUM      40 // OK CONNECTION ON BOARD
#define Y2_GPIO_NUM      39 // OK CONNECTION ON BOARD

#define WARMUP_TARGET_FRAMES 30
#define WARMUP_TIMEOUT_MS    10000
#define MAX_CONSECUTIVE_NULL_FRAMES 5
#define WARMUP_FRAME_DELAY_MS 50

// DIAGNOSTIC (temporary): when 1, reboot via esp_restart() instead of deep sleep.
// Used to isolate whether the camera's 0-frames-after-wake is deep-sleep-specific
// SoC state (camera works after a plain reboot) or hardware (still fails). Revert
// to 0 for normal low-power operation — esp_restart does NOT sleep.
#ifndef CAM_DEBUG_RESTART_INSTEAD_OF_SLEEP
#define CAM_DEBUG_RESTART_INSTEAD_OF_SLEEP 0
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 800
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 600
#define EI_CAMERA_FRAME_BYTE_SIZE 3

#define MAX_CYCLE_TIME_MS 120000
static unsigned long cycleStartTime = 0;
static bool isCycleTimeoutExceeded(void) {
  if (cycleStartTime == 0) return false;
  return (millis() - cycleStartTime > MAX_CYCLE_TIME_MS);
}

__attribute__((weak)) void* ei_malloc(size_t size) {
  if (size == 0) return nullptr;
  if (size > 1024) {
    void* p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p) return p;
  }
  return heap_caps_aligned_alloc(16, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
__attribute__((weak)) void* ei_calloc(size_t nitems, size_t size) {
  size_t total = nitems * size;
  if (total == 0) return nullptr;
  void* p = nullptr;
  if (total > 1024) p = heap_caps_aligned_alloc(16, total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = heap_caps_aligned_alloc(16, total, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (p) memset(p, 0, total);
  return p;
}
__attribute__((weak)) void ei_free(void* ptr) { heap_caps_free(ptr); }

#define ei_printf(...) do { Serial.printf(__VA_ARGS__); } while(0)
#define ei_sleep(ms) (vTaskDelay(pdMS_TO_TICKS(ms)), EI_IMPULSE_OK)

uint8_t* snapshot_buf = nullptr;
bool is_initialised = false;
float levelConfidence = 0.0f;
float materialConfidence = 0.0f;
float fillLevel = -1.0f;
String detectedMaterial;
float materialClassConfidences[13] = {0};
float overallConfidence = 0.0f;
static bool debug_nn = false;
uint8_t* classificationPhoto = nullptr;
size_t classificationPhotoSize = 0;
bool doCameraTCPJob = true;
int topCropPercent = 40;

// Phase 2b: sleep and modem
#define uS_TO_S_FACTOR 1000000ULL
#define DEFAULT_SLEEP_TIME 3600  // seconds
RTC_DATA_ATTR int customSleepDuration = DEFAULT_SLEEP_TIME;
// Whether the upcoming daytime sleep should arm PIR wake. Set per geofence state
// by configureDaytimeSleepForState() from the state's pir_home/away_mode; the
// no-fix/retry paths leave it at the default (armed) so motion can still wake.
static bool s_daytimePirArm = true;

// Operational window (local time). Active between WAKE and NAP; sleeps outside.
// Handles midnight-spanning sleep windows (e.g. NAP 20:00 → WAKE 06:00).
#define WAKE_HOUR   7
#define WAKE_MINUTE 0
#define NAP_HOUR   20
#define NAP_MINUTE  0
// Fallback TZ offset in quarter-hours from UTC, used when modem CCLK lacks a TZ suffix.
// CET (UTC+1) = 4, CEST (UTC+2) = 8.
#define FALLBACK_TZ_QUARTER_HOURS 0
// FIELD FINDING 2026-07-29 (fw v9 regression): the Czech NB-IoT operator
// delivers CCLK in LOCAL time with NO TZ suffix — not UTC as assumed. v9
// enabled the EU-DST auto fallback below, which added CEST (+2 h) on top of
// already-local time: the device slept 20:00–04:00 instead of 22:00–06:00
// (confirmed from server-side check-in timestamps, both edges shifted −2 h).
// Default is therefore 0: treat suffix-less CCLK as already-local, exactly
// like fw ≤v8. Enable (build flag) ONLY for a network proven to send UTC,
// e.g. by comparing CCLK against GNSS time on the bench.
#ifndef FALLBACK_TZ_AUTO_CZ
#define FALLBACK_TZ_AUTO_CZ 0
#endif

#define BC95_RX_PIN 44  // ESP32 RX connected to BC95 TX
#define BC95_TX_PIN 43  // ESP32 TX connected to BC95 RX

// Modem UART baud. Raised from 9600 to 115200 so OTA (~2.3 MB) downloads in
// ~4 min instead of ~45 min. The firmware auto-detects the modem's current baud
// on boot and raises it (AT+IPR) when it is still at the fallback — so a fresh
// module, or one reset by AT&F0, is migrated transparently. Drop to 9600 (set
// BC95_BAUD to BC95_BAUD_FALLBACK) if your wiring can't sustain 115200.
#ifndef BC95_BAUD
#define BC95_BAUD 115200
#endif
#ifndef BC95_BAUD_FALLBACK
#define BC95_BAUD_FALLBACK 9600
#endif
// Factory-reset (AT&F0) the modem during the "not registered" full-init path. Default OFF:
// AT&F0 wipes the persisted UART baud (back to 9600 → 30-attempt fallback next boot) AND the
// APN, so a board that merely lost coverage would re-provision from scratch every boot. With
// it OFF, baud/APN/PSM persist (AT&W) and an unregistered board just re-attaches at 115200.
// Turn ON only to force a clean re-provision of a misbehaving/used module.
#ifndef BC95_FACTORY_RESET_ON_PROVISION
#define BC95_FACTORY_RESET_ON_PROVISION 0
#endif
// Bench flag: skip all modem uplink work in the main job (registration, telemetry TCP, OTA)
// so the device goes GNSS → sleep quickly. Lets you capture a clean GNSS sleep-current trace
// without the modem registration churn (which can add minutes of awake time on a SIM/coverage
// failure). GNSS acquisition + the EXTINT standby/wake path still run. Bench only — no uplink.
#ifndef BENCH_SKIP_UPLINK
#define BENCH_SKIP_UPLINK 0
#endif

// HC-SR501 PIR OUT (3.3 V, active HIGH) — deep-sleep wake on motion during daytime only.
#define PIR_WAKE_GPIO 10
#ifndef PIR_WAKE_ENABLE
#define PIR_WAKE_ENABLE 1
#endif
#define PIR_WAKE_SETTLE_MS 100    // OUT must stay LOW this long before arming EXT1
#define PIR_WAKE_ARM_QUIET_MS 200 // wait for quiet line before sampling
#define PIR_WAKE_CONFIRM_MS 120   // OUT must stay HIGH this long after wake to count as motion
HardwareSerial bc95_modem(2);
static const char* SERVER_IP = "34.10.203.180";
static const int TCP_PORT = 8009;
static bool moduleInitialized = false;

// TCP receive: optional wait for server response after sending data (AT+QIRD)
#define TCP_WAIT_FOR_SERVER_RESPONSE 1  // 0 = one-way upload only; 1 = wait for server JSON/command
// Enabled so the server can push runtime config (set_config) + OTA replies after
// each image upload. Adds ~1-2s awake per cycle waiting for the reply.
#define TCP_RECV_TIMEOUT_MS 15000   // How long to wait for server response
#define TCP_RECV_POLL_MS    1000    // Poll interval for AT+QIRD
#define TCP_RECV_BUFFER_SIZE 512    // Max command size from server

// ---- OTA over NB-IoT (raw TCP, length-framed). See OTA-NB-IoT.md ----
#ifndef FW_VERSION
#define FW_VERSION 1            // running firmware version (also settable via build_flags)
#endif
#ifndef OTA_SERVER_IP
#define OTA_SERVER_IP SERVER_IP // OTA server host (default: same VM as telemetry)
#endif
#ifndef OTA_TCP_PORT
#define OTA_TCP_PORT 8010       // OTA server port (telemetry server stays on TCP_PORT)
#endif
// Check for a new image on every true power-on (cold boot). Handy for bench
// testing: power-cycle the board to trigger an update check. Set to 0 for
// production and drive updates with the server {"cmd":"ota"} command instead.
#ifndef OTA_CHECK_ON_COLD_BOOT
#define OTA_CHECK_ON_COLD_BOOT 1
#endif
// Honor a server {"cmd":"ota"} reply (requires TCP_WAIT_FOR_SERVER_RESPONSE=1).
#ifndef OTA_VIA_SERVER_COMMAND
#define OTA_VIA_SERVER_COMMAND 1
#endif
#ifndef OTA_READ_CHUNK_TIMEOUT_MS
#define OTA_READ_CHUNK_TIMEOUT_MS 120000UL  // per-chunk QIRD budget (NB-IoT is slow)
#endif
#ifndef OTA_DOWNLOAD_BUDGET_MS
#define OTA_DOWNLOAD_BUDGET_MS (60UL * 60UL * 1000UL)  // 60 min max (9600 baud ~44 min + margin)
#endif
// Poll interval while the modem RX buffer is momentarily empty mid-download.
// Shorter than the 1 s telemetry poll so we keep up with the server's pacing.
#ifndef OTA_RX_POLL_MS
#define OTA_RX_POLL_MS 300
#endif
// Bytes per AT+QIRD. MUST match the modem's socket read cap: this module rejects
// larger read lengths with ERROR (the telemetry path uses TCP_RECV_BUFFER_SIZE=512
// and the OTA header read uses 512 — both work; 1200 made QIRD return ERROR).
#ifndef OTA_QIRD_READ_LEN
#define OTA_QIRD_READ_LEN 512
#endif
static volatile bool g_otaRequested = false;
#if OTA_CHECK_ON_COLD_BOOT
static bool g_otaColdBootPending = false;
#endif

static bool usePSM = true;  // Keep modem in PSM during ESP deep sleep to minimize current (modem supply stays on)
#define PSM_ACTIVE_TIME "00100001"   // 1 minute
#define PSM_PERIODIC_TAU "00100001"  // 1 hour
static bool useEDRX = false;  // eDRX + PSM together raises sleep current; PSM-only for min power
#define EDRX_CYCLE "0011"  // 40.96 seconds

static uint64_t device_id = 0;
static char measurement_id[37] = {0};

// Telemetry stubs for buildLineProtocolASCII (sensors can be added later)
static int batteryPercentage = 0;
static bool sensorDataValid = false;
static struct { float temperature = 0.0f; float humidity = 0.0f; } sensorData;
// ---- Hardware variant switches (compile-time) ----
#ifndef HAS_GNSS
#define HAS_GNSS 0
#endif

// GNSS (u-blox MAX-M10S) on I2C (DDC)
#define GNSS_I2C_ADDR 0x42

// --- EXTINT hot-start wake (requires PCB rework) ---
// Per the MAX-M10S integration manual, software standby (UBX-RXM-PMREQ) can only be woken by
// UART RX or EXTINT — not by DDC/I2C. On the stock board EXTINT (U15-5) is driven by the
// open-drain antenna-supervisor buffer U16-Y1 (no pull-up); when standby drops LNA_EN the
// antenna-sense node decays and toggles EXTINT, waking the receiver back to ~24 mA tracking
// (~8 s in). HW fix: cut EXTINT from U16-Y1 and wire it to GNSS_WAKE_PIN. Firmware then holds
// EXTINT LOW (idle) through deep sleep — no spurious wake — and pulses it HIGH to wake the
// receiver from standby WITHOUT clearing BBR, preserving hot start (V_IO keeps BBR alive).
// Leave at 0 on un-reworked boards. When 1, the PMREQ also arms EXTINT0 as the wake source.
#ifndef GNSS_EXTINT_WAKE
#define GNSS_EXTINT_WAKE 0
#endif
#ifndef GNSS_WAKE_PIN
#define GNSS_WAKE_PIN 16   // bodge to U15-5 EXTINT. RTC GPIOs (0..21) hold via rtc_gpio_hold;
#endif                     // non-RTC pins (e.g. GPIO38) hold via gpio_hold + deep-sleep hold.
#ifndef GNSS_EXTINT_WAKE_SETTLE_MS
#define GNSS_EXTINT_WAKE_SETTLE_MS 120UL  // hold EXTINT high while RF/clock spin up after wake
#endif

// Bench diagnostic for the GNSS deep-sleep current investigation. When 1, the sleep
// path logs the I2C ack of 0x42/0x3C/0x36 with all rails on, and of 0x42 again after
// the camera rail is cut. The post-cut probe re-touches the bus and WILL wake the M10
// from backup, so this is for root-causing the clamp only — never ship with it set.
#ifndef GNSS_SLEEP_DIAG
#define GNSS_SLEEP_DIAG 0
#endif

// Bench diagnostic: keep the camera rail (LD_SWCH_1) POWERED through deep sleep instead of
// cutting it, so the shared I2C bus cannot be clamped by the decaying/unpowered camera.
// If the M10 then stays in software backup for the whole sleep window (measured floor ~µA
// above the camera's idle current, not ~24 mA tracking), the camera-rail glitch is the
// confirmed wake source and hardware bus isolation is the fix. Costs camera idle current —
// this is NOT a low-power production setting, only a root-cause confirmation build.
#ifndef GNSS_SLEEP_NO_RAIL_CUT
#define GNSS_SLEEP_NO_RAIL_CUT 0
#endif

// Bench diagnostic: for one sleep cycle, power the BC95 RF fully off (AT+CFUN=0) instead of
// configuring PSM, and force timer-only sleep (no PIR). Camera rail is cut as normal. This
// removes the modem (its DRX paging RF bursts and PSM-entry transient) as a variable. If the
// M10 then stays in software backup (floor << 1 mA, no ~24 mA step ~7-9 s in), the modem is
// the confirmed wake source. If it still steps to ~24 mA, the modem is exonerated. Not a
// production setting — the modem is left RF-off, so no uplink on the next wake until re-init.
#ifndef GNSS_SLEEP_MODEM_OFF_TEST
#define GNSS_SLEEP_MODEM_OFF_TEST 0
#endif

#ifndef GNSS_ACQUIRE_TIMEOUT_MS
#define GNSS_ACQUIRE_TIMEOUT_MS 60000UL
#endif

#ifndef GNSS_NO_FIX_RETRY_S
#define GNSS_NO_FIX_RETRY_S 600  // 10 minutes — retry GNSS / geofence without camera/ML
#endif

// Hot-start tuning (module on permanent 3.3V keeps tracking across sleep).
#ifndef GNSS_DRAIN_MAX_MS
#define GNSS_DRAIN_MAX_MS 250UL   // max time to drain stale DDC backlog after wake
#endif
#ifndef GNSS_META_GRACE_MS
#define GNSS_META_GRACE_MS 1500UL // after first position, wait this long for GGA sats/HDOP
#endif
#ifndef GNSS_POWER_PERMANENT
#define GNSS_POWER_PERMANENT 1  // 1 = jumper 3V1; 0 = jumper VDD1 (GNSS on switched VDD rail)
#endif

struct GnssFix {
  bool valid = false;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  float speed_knots = 0.0f;
  float course_deg = 0.0f;
  float altitude_m = 0.0f;
  int fix_quality = 0;
  int sats_used = 0;
  float hdop = 0.0f;
  int utc_hh = -1, utc_mm = -1, utc_ss = -1;
  int utc_DD = -1, utc_MM = -1, utc_YY = -1;
};

static GnssFix g_gnssLastFix;

// Per-state PIR wake mode (in-depot "home" and deployed "away" daytime sleep).
enum PirMode : uint8_t {
  PIR_MODE_PIR_TIMER = 0,  // wake on motion OR after sleep_X_s (current behaviour)
  PIR_MODE_TIMER_ONLY = 1, // ignore PIR, wake only after sleep_X_s
  PIR_MODE_PIR_ONLY   = 2, // wake only on motion (timer capped at PIR_ONLY_SAFETY_S)
};
static const uint32_t PIR_ONLY_SAFETY_S = 86400;  // worst-case wake bound for PIR-only

// Home geofence + daytime sleep intervals + operational window + PIR modes
// (NVS-backed, see loadSleepGeoFromNvs). Magic bumped whenever fields change so
// old blobs are discarded and defaults re-applied (then the server pushes the
// desired values via set_config).
static const uint32_t kSleepGeoMagic = 0x53474532u;  // 'SGE2' (was 'SGEP' before wake/nap/pir fields)
#pragma pack(push, 1)
struct SleepGeoConfig {
  uint32_t magic = 0;
  double home_lat = 0;
  double home_lon = 0;
  uint32_t home_radius_m = 0;   // 0 = geofence disabled
  uint32_t sleep_home_s = 0;
  uint32_t sleep_away_s = 0;
  // Operational window (local time), formerly the WAKE_/NAP_ compile-time macros.
  uint8_t wake_hour = 0;
  uint8_t wake_minute = 0;
  uint8_t nap_hour = 0;
  uint8_t nap_minute = 0;
  uint8_t pir_home_mode = PIR_MODE_PIR_TIMER;  // PirMode
  uint8_t pir_away_mode = PIR_MODE_PIR_TIMER;  // PirMode
};
#pragma pack(pop)

static SleepGeoConfig g_sleepGeo;
/** Short tag for OLED line during daytime sleep (set by applyDaytimeSleepFromGeofence). */
static char s_sleepOledExtra[16] = {0};

static bool useFuelGauge = true;
#define MAX17048_I2C_ADDR 0x36

#ifndef I2C_CLOCK_HZ
#define I2C_CLOCK_HZ 100000UL
#endif

// Phase 1c: I2C and OLED
static bool i2c_bus_initialized = false;

static void i2cBusRecover(void) {
  Serial.println("Attempting I2C bus recovery...");
  pinMode(SIOD_GPIO_NUM, INPUT);
  if (digitalRead(SIOD_GPIO_NUM) == LOW) {
    Serial.println("SDA stuck LOW, toggling SCL to release...");
    pinMode(SIOC_GPIO_NUM, OUTPUT);
    for (int i = 0; i < 16; i++) {
      digitalWrite(SIOC_GPIO_NUM, LOW);
      delayMicroseconds(5);
      digitalWrite(SIOC_GPIO_NUM, HIGH);
      delayMicroseconds(5);
      if (digitalRead(SIOD_GPIO_NUM) == HIGH) {
        Serial.printf("I2C bus released after %d SCL toggles\n", i + 1);
        break;
      }
    }
    pinMode(SIOD_GPIO_NUM, OUTPUT);
    digitalWrite(SIOD_GPIO_NUM, LOW);
    delayMicroseconds(5);
    digitalWrite(SIOC_GPIO_NUM, HIGH);
    delayMicroseconds(5);
    digitalWrite(SIOD_GPIO_NUM, HIGH);
    delayMicroseconds(5);
    pinMode(SIOD_GPIO_NUM, INPUT);
    pinMode(SIOC_GPIO_NUM, INPUT);
  } else {
    Serial.println("SDA is HIGH, bus appears OK");
  }
}

static void initI2CBusOnce(void) {
  if (i2c_bus_initialized) return;
  Serial.printf("Initializing I2C bus (SDA: GPIO%d, SCL: GPIO%d, %lu Hz)...\n",
                SIOD_GPIO_NUM, SIOC_GPIO_NUM, (unsigned long)I2C_CLOCK_HZ);
  i2cBusRecover();
  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire.setClock(I2C_CLOCK_HZ);
  i2c_bus_initialized = true;
  Serial.println("I2C bus initialized");
}


// OLED: SSD1315 compatible with SSD1306 driver, 128x64, HW I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

static bool showOLED = true;
/** True only after u8g2.begin() succeeds — PSM/modem helpers call oled_status; must no-op before display init. */
static bool s_oled_initialized = false;

struct OLEDStatus {
  // Top bar (updated when modem/sensors available)
  int signalRSSI = 99;      // 0-31 or 99=unknown (CSQ)
  int networkHour = -1;     // -1=unknown, 0-23
  int networkMinute = -1;   // -1=unknown, 0-59
  int batteryPercent = -1;  // -1=unknown, 0-100%
  // Status lines and progress
  String statusLine1;
  String statusLine2;
  int progress = 0;
  bool showProgress = false;
  // Classification
  float fillLevel = -1.0f;
  String material;
  float lvlConf = -1.0f;
  float matConf = -1.0f;
  int imageWidth = 0;
  int imageHeight = 0;
} oledStatus;

static void oled_status(const char* line1, const char* line2, int progress);

static void oled_layout_main(void) {
  oled_status("Ready", "", -1);
}

static void oled_update(void) {
  if (!showOLED) return;
  if (!s_oled_initialized) return;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Line 1 (y=8): Status bar - Signal, Time, Battery
  int bars = 0;
  if (oledStatus.signalRSSI != 99 && oledStatus.signalRSSI >= 0) {
    if (oledStatus.signalRSSI >= 25) bars = 5;
    else if (oledStatus.signalRSSI >= 20) bars = 4;
    else if (oledStatus.signalRSSI >= 15) bars = 3;
    else if (oledStatus.signalRSSI >= 10) bars = 2;
    else if (oledStatus.signalRSSI >= 5) bars = 1;
  }
  char sigStr[12];
  snprintf(sigStr, sizeof(sigStr), "S[%s%s%s%s%s]",
           bars >= 1 ? "|" : "-", bars >= 2 ? "|" : "-", bars >= 3 ? "|" : "-",
           bars >= 4 ? "|" : "-", bars >= 5 ? "|" : "-");
  u8g2.drawStr(0, 8, sigStr);

  char timeStr[8];
  if (oledStatus.networkHour >= 0 && oledStatus.networkMinute >= 0) {
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", oledStatus.networkHour, oledStatus.networkMinute);
  } else {
    snprintf(timeStr, sizeof(timeStr), "--:--");
  }
  u8g2.drawStr(52, 8, timeStr);

  char batStr[8];
  if (oledStatus.batteryPercent >= 0) {
    snprintf(batStr, sizeof(batStr), "B%d%%", oledStatus.batteryPercent);
  } else {
    snprintf(batStr, sizeof(batStr), "B--%%");
  }
  int batWidth = u8g2.getStrWidth(batStr);
  u8g2.drawStr(128 - batWidth, 8, batStr);

  u8g2.drawHLine(0, 11, 128);

  char fillStr[24];
  if (oledStatus.fillLevel >= 0 && oledStatus.lvlConf >= 0) {
    snprintf(fillStr, sizeof(fillStr), "FILL: %.0f%% / %.2f", oledStatus.fillLevel, oledStatus.lvlConf);
  } else {
    snprintf(fillStr, sizeof(fillStr), "FILL: --.- / --");
  }
  u8g2.drawStr(0, 22, fillStr);
  char matStr[24];
  if (oledStatus.material.length() > 0 && oledStatus.matConf >= 0) {
    String mat = oledStatus.material;
    if (mat.length() > 10) mat = mat.substring(0, 10);
    mat.toUpperCase();
    snprintf(matStr, sizeof(matStr), "MAT : %s / %.2f", mat.c_str(), oledStatus.matConf);
  } else {
    snprintf(matStr, sizeof(matStr), "MAT : ---- / --");
  }
  u8g2.drawStr(0, 32, matStr);
  if (oledStatus.statusLine1.length() > 0) {
    u8g2.drawStr(0, 43, oledStatus.statusLine1.c_str());
  }
  if (oledStatus.statusLine2.length() > 0) {
    u8g2.drawStr(0, 53, oledStatus.statusLine2.c_str());
  }
  if (oledStatus.showProgress) {
    u8g2.drawFrame(0, 55, 128, 9);
    int fillWidth = (oledStatus.progress * 126) / 100;
    if (fillWidth > 0) {
      u8g2.drawBox(1, 56, fillWidth, 7);
    }
  }
  u8g2.sendBuffer();
}

static void oled_status(const char* line1, const char* line2 = "", int progress = -1) {
  if (!s_oled_initialized) return;
  oledStatus.statusLine1 = line1;
  oledStatus.statusLine2 = line2;
  if (progress >= 0) {
    oledStatus.progress = progress;
    oledStatus.showProgress = true;
  } else {
    oledStatus.showProgress = false;
  }
  oled_update();
}

static void oled_draw_splash_logo(void) {
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 10, " _     _                 _");
  u8g2.drawStr(0, 18, "(_)___| |_  _ __  __ _ _(_)__");
  u8g2.drawStr(0, 26, "| / _ \\  _|| '  \\/ _` / _` / _|");
  u8g2.drawStr(0, 34, "|_\\___/\\__||_|_|_\\__,_\\__, \\__|");
  u8g2.drawStr(0, 42, "                      |___/");
}

static void oled_splash(void) {
  u8g2.clearBuffer();
  oled_draw_splash_logo();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(30, 58, "Container AI v1.2");
  u8g2.sendBuffer();
}

/**
 * Animated boot/loading screen: logo + a bouncing "working" bar so it is
 * obvious the device powered up and is alive (not frozen). Blocks for
 * ~durationMs, redrawing every frame. No-op until the display is initialized.
 */
static void oled_boot_loading(uint32_t durationMs) {
  if (!s_oled_initialized) return;
  const int barW = 28;
  const int range = 128 - barW;  // bounce travel distance
  const uint32_t start = millis();
  uint32_t frame = 0;
  while (millis() - start < durationMs) {
    u8g2.clearBuffer();
    oled_draw_splash_logo();

    // Brand line just under the logo.
    u8g2.setFont(u8g2_font_5x7_tf);
    const char* brand = "Container AI v1.2";
    u8g2.drawStr((128 - u8g2.getStrWidth(brand)) / 2, 52, brand);

    // Indeterminate (bouncing) progress bar = continuous motion so the
    // operator can see the device is running even while init blocks.
    u8g2.drawFrame(0, 56, 128, 8);
    int pos = (int)((frame * 8) % (uint32_t)(2 * range));
    if (pos > range) pos = 2 * range - pos;  // reflect for back-and-forth sweep
    u8g2.drawBox(pos + 1, 58, barW, 4);

    u8g2.sendBuffer();
    frame++;
    delay(70);
  }
}

/**
 * Bring the OLED up as early as possible. The shared GPIO1/2 I2C bus must
 * already be initialized (initI2CBusOnce()). Safe to call more than once.
 * This lets the loading screen appear *before* the long pre-camera GNSS
 * acquire instead of after it. Returns true if the panel is usable.
 */
static bool oled_bring_up_early(void) {
  if (s_oled_initialized) return true;
  if (!u8g2.begin()) {
    Serial.println("Early OLED init failed (will retry after camera)");
    return false;
  }
  s_oled_initialized = true;
  Serial.println("OLED up early (pre-GNSS loading screen)");
  return true;
}

/** Progress callback type for blocking GNSS reads (elapsed/timeout in ms). */
typedef void (*GnssProgressCb)(unsigned long elapsedMs, unsigned long timeoutMs);

/**
 * One frame of the full-screen "searching for GNSS" animation: a globe with a
 * satellite orbiting it, a pulsing search ring, an animated caption, and a
 * timeout countdown bar. Phase is derived from elapsedMs so it animates smoothly.
 */
static void oled_gnss_search_frame(unsigned long elapsedMs, unsigned long timeoutMs) {
  if (!s_oled_initialized) return;
  const int cx = 64;
  const int cy = 33;
  const int planetR = 11;
  const float rx = 44.0f;   // orbit ellipse half-width
  const float ry = 17.0f;   // orbit ellipse half-height

  u8g2.clearBuffer();

  // Caption with animated dots (top).
  u8g2.setFont(u8g2_font_6x10_tf);
  int dots = (int)((elapsedMs / 400) % 4);
  char msg[24];
  snprintf(msg, sizeof(msg), "Searching GPS%.*s", dots, "...");
  u8g2.drawStr((128 - u8g2.getStrWidth(msg)) / 2, 9, msg);

  // Dashed orbit ellipse.
  for (int a = 0; a < 360; a += 15) {
    float ar = a * 0.01745329f;
    int ox = cx + (int)(rx * cosf(ar));
    int oy = cy + (int)(ry * sinf(ar));
    u8g2.drawPixel(ox, oy);
  }

  // Pulsing search ring emanating from the planet.
  int pulse = (int)((elapsedMs / 110) % 14);
  if (pulse <= 10) {
    u8g2.drawCircle(cx, cy, planetR + 2 + pulse, U8G2_DRAW_ALL);
  }

  // Planet: filled globe with black equator + meridian for a 3D look.
  u8g2.drawDisc(cx, cy, planetR);
  u8g2.setDrawColor(0);
  u8g2.drawEllipse(cx, cy, planetR - 1, planetR / 3, U8G2_DRAW_ALL);  // equator
  u8g2.drawEllipse(cx, cy, planetR / 3, planetR - 1, U8G2_DRAW_ALL);  // meridian
  u8g2.setDrawColor(1);

  // Orbiting satellite (body + solar wings + antenna).
  float ang = (elapsedMs / 1000.0f) * 2.4f;
  int sx = cx + (int)(rx * cosf(ang));
  int sy = cy + (int)(ry * sinf(ang));
  u8g2.drawDisc(sx, sy, 2);
  u8g2.drawHLine(sx - 4, sy, 9);
  u8g2.drawPixel(sx, sy - 3);

  // Timeout countdown bar (bottom).
  if (timeoutMs > 0) {
    int w = (int)((elapsedMs * 126) / timeoutMs);
    if (w > 126) w = 126;
    u8g2.drawFrame(0, 57, 128, 7);
    if (w > 0) u8g2.drawBox(1, 58, w, 5);
  }

  u8g2.sendBuffer();
}

/**
 * Throttled progress callback for gnssReadLatestFixSinceWake(): redraws the
 * search animation a few times per second without starving the GNSS DDC reads.
 */
static void gnssSearchAnimTick(unsigned long elapsedMs, unsigned long timeoutMs) {
  static unsigned long s_lastAnimMs = 0;
  unsigned long now = millis();
  if (now - s_lastAnimMs < 90) return;   // cap at ~11 fps
  s_lastAnimMs = now;
  oled_gnss_search_frame(elapsedMs, timeoutMs);
  Wire.setClock(I2C_CLOCK_HZ);           // restore GNSS bus clock after OLED writes
}

static void highz_no_pulls(int g);

// ---- GNSS (u-blox MAX-M10S via DDC/I2C) ----

static bool nmeaChecksumOK(const String& line) {
  int star = line.indexOf('*');
  if (star < 0) return true;
  if (star + 2 >= (int)line.length()) return false;
  uint8_t calc = 0;
  int start = (line.length() > 0 && line[0] == '$') ? 1 : 0;
  for (int i = start; i < star; i++) calc ^= (uint8_t)line[i];
  char h1 = line[star + 1];
  char h2 = line[star + 2];
  auto hexVal = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  int v1 = hexVal(h1), v2 = hexVal(h2);
  if (v1 < 0 || v2 < 0) return false;
  uint8_t got = (uint8_t)((v1 << 4) | v2);
  return got == calc;
}

static double nmeaDegMinToDecimal(const String& ddmm, const String& hemi) {
  if (ddmm.length() < 4) return 0.0;
  int dot = ddmm.indexOf('.');
  int minStart = (dot == -1) ? (ddmm.length() - 2) : (dot - 2);
  if (minStart <= 0) return 0.0;
  double deg = ddmm.substring(0, minStart).toDouble();
  double minutes = ddmm.substring(minStart).toDouble();
  double dec = deg + minutes / 60.0;
  if (hemi == "S" || hemi == "W") dec = -dec;
  return dec;
}

static bool splitNmeaCSV(const String& body, String* out, int maxFields, int& outCount) {
  outCount = 0;
  int start = 0;
  while (outCount < maxFields) {
    int comma = body.indexOf(',', start);
    if (comma == -1) {
      out[outCount++] = body.substring(start);
      return true;
    }
    out[outCount++] = body.substring(start, comma);
    start = comma + 1;
  }
  return true;
}

static void gnssApplyRMC(const String& body, GnssFix& fix) {
  String f[16];
  int n = 0;
  splitNmeaCSV(body, f, 16, n);
  if (n < 10) return;
  const String& t = f[1];
  const String& status = f[2];
  const String& lat = f[3];
  const String& latH = f[4];
  const String& lon = f[5];
  const String& lonH = f[6];
  const String& spd = f[7];
  const String& crs = f[8];
  const String& date = f[9];

  if (t.length() >= 6) {
    fix.utc_hh = t.substring(0, 2).toInt();
    fix.utc_mm = t.substring(2, 4).toInt();
    fix.utc_ss = t.substring(4, 6).toInt();
  }
  if (date.length() >= 6) {
    fix.utc_DD = date.substring(0, 2).toInt();
    fix.utc_MM = date.substring(2, 4).toInt();
    fix.utc_YY = date.substring(4, 6).toInt();
  }

  if (status == "A") {
    if (lat.length() > 0 && lon.length() > 0) {
      fix.lat_deg = nmeaDegMinToDecimal(lat, latH);
      fix.lon_deg = nmeaDegMinToDecimal(lon, lonH);
    }
    fix.speed_knots = spd.toFloat();
    fix.course_deg = crs.toFloat();
    if (std::fabs(fix.lat_deg) > 0.001 || std::fabs(fix.lon_deg) > 0.001) fix.valid = true;
  }
}

static void gnssApplyGGA(const String& body, GnssFix& fix) {
  String f[20];
  int n = 0;
  splitNmeaCSV(body, f, 20, n);
  if (n < 10) return;
  const String& lat = f[2];
  const String& latH = f[3];
  const String& lon = f[4];
  const String& lonH = f[5];
  fix.fix_quality = f[6].toInt();
  fix.sats_used = f[7].toInt();
  fix.hdop = f[8].toFloat();
  fix.altitude_m = f[9].toFloat();

  if (fix.fix_quality > 0) {
    if (lat.length() > 0 && lon.length() > 0) {
      fix.lat_deg = nmeaDegMinToDecimal(lat, latH);
      fix.lon_deg = nmeaDegMinToDecimal(lon, lonH);
    }
    if (std::fabs(fix.lat_deg) > 0.001 || std::fabs(fix.lon_deg) > 0.001) fix.valid = true;
  }
}

static size_t gnssDdcReadPayload(uint8_t* out, size_t outMax, uint16_t* availOut = nullptr) {
  if (outMax == 0) return 0;
  size_t req = 2 + outMax;
  if (req > 255) req = 255;

  int n = Wire.requestFrom((uint8_t)GNSS_I2C_ADDR, (uint8_t)req, (uint8_t)true);
  if (n < 2) return 0;

  uint8_t msb = (uint8_t)Wire.read();
  uint8_t lsb = (uint8_t)Wire.read();
  uint16_t avail = (uint16_t)((msb << 8) | lsb);
  if (availOut) *availOut = avail;

  size_t toCopy = min((size_t)avail, outMax);
  size_t got = 0;
  while (Wire.available() && got < toCopy) {
    out[got++] = (uint8_t)Wire.read();
  }
  while (Wire.available()) (void)Wire.read();

  return got;
}

/** Send a UBX frame over DDC (I2C). Checksum is computed here, not hand-typed.
 * Returns the Wire.endTransmission() status (0 = ACK/OK, non-zero = NACK/error). */
static uint8_t gnssSendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t frame[8 + 32];
  if (len > 32) return 0xFF;
  size_t n = 0;
  frame[n++] = 0xB5;                       // sync 1
  frame[n++] = 0x62;                       // sync 2
  frame[n++] = cls;
  frame[n++] = id;
  frame[n++] = (uint8_t)(len & 0xFF);
  frame[n++] = (uint8_t)((len >> 8) & 0xFF);
  for (uint16_t i = 0; i < len; i++) frame[n++] = payload[i];
  uint8_t ckA = 0, ckB = 0;                // Fletcher over class..payload
  for (size_t i = 2; i < n; i++) { ckA = (uint8_t)(ckA + frame[i]); ckB = (uint8_t)(ckB + ckA); }
  frame[n++] = ckA;
  frame[n++] = ckB;
  Wire.beginTransmission(GNSS_I2C_ADDR);
  Wire.write(frame, n);
  return Wire.endTransmission(true);
}

#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_EXTINT_WAKE
/** Drive the rewired EXTINT line to its idle (no-wake) LOW level, latched across deep sleep. */
static void gnssExtintIdleAndHold(void) {
  if (rtc_gpio_is_valid_gpio((gpio_num_t)GNSS_WAKE_PIN)) rtc_gpio_hold_dis((gpio_num_t)GNSS_WAKE_PIN);
  else gpio_hold_dis((gpio_num_t)GNSS_WAKE_PIN);
  pinMode(GNSS_WAKE_PIN, OUTPUT);
  digitalWrite(GNSS_WAKE_PIN, LOW);
  if (rtc_gpio_is_valid_gpio((gpio_num_t)GNSS_WAKE_PIN)) rtc_gpio_hold_en((gpio_num_t)GNSS_WAKE_PIN);
  else gpio_hold_en((gpio_num_t)GNSS_WAKE_PIN);
}

/** Release the latch and leave EXTINT idle LOW as a normal output (call once early on boot). */
static void gnssExtintInit(void) {
  if (rtc_gpio_is_valid_gpio((gpio_num_t)GNSS_WAKE_PIN)) rtc_gpio_hold_dis((gpio_num_t)GNSS_WAKE_PIN);
  else gpio_hold_dis((gpio_num_t)GNSS_WAKE_PIN);
  pinMode(GNSS_WAKE_PIN, OUTPUT);
  digitalWrite(GNSS_WAKE_PIN, LOW);
}

/** Pulse EXTINT HIGH to wake the M10 from software standby (hot start — BBR is preserved). */
static void gnssExtintWakePulse(void) {
  if (rtc_gpio_is_valid_gpio((gpio_num_t)GNSS_WAKE_PIN)) rtc_gpio_hold_dis((gpio_num_t)GNSS_WAKE_PIN);
  else gpio_hold_dis((gpio_num_t)GNSS_WAKE_PIN);
  pinMode(GNSS_WAKE_PIN, OUTPUT);
  digitalWrite(GNSS_WAKE_PIN, LOW);
  delay(2);
  digitalWrite(GNSS_WAKE_PIN, HIGH);          // rising edge / high level = wake
  delay(GNSS_EXTINT_WAKE_SETTLE_MS);          // stay high while RF + clock spin up
  digitalWrite(GNSS_WAKE_PIN, LOW);           // back to idle (no-wake) level
  Serial.printf("[GNSS] EXTINT wake pulse on GPIO%d (hot-start)\n", GNSS_WAKE_PIN);
}
#endif

/**
 * Nudge the MAX-M10S out of software backup. With the EXTINT rework (GNSS_EXTINT_WAKE) a
 * rising edge on EXTINT is the spec-valid wake from software standby; the DDC byte below is
 * only kept to (re)set the read-stream register pointer afterwards. Without the rework, the
 * DDC write is the (best-effort) nudge. Either way a settle delay lets the RF front-end and
 * clock spin back up before we start reading NMEA.
 */
static void gnssWakeFromBackup(void) {
#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_EXTINT_WAKE
  gnssExtintWakePulse();
#endif
  Wire.beginTransmission(GNSS_I2C_ADDR);
  Wire.write((uint8_t)0xFF);
  Wire.endTransmission(true);
  delay(120);
}

/**
 * Put the (permanently-powered) MAX-M10S into software backup for the sleep window.
 * M10 UBX-RXM-PMREQ flags (see u-blox M10 interface description): bit1=backup,
 * bit2=force — both are required for minimum sleep current (~µA on V_IO, not ~25 mA).
 *
 * MUST be the last I2C activity before esp_deep_sleep_start(). Any later SDA/SCL
 * toggling (including driving the lines LOW) wakes the module back to tracking.
 */
static void gnssEnterBackup(void) {
  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(2);
  uint8_t pmreq[16] = {
    0x00,                    // version
    0x00, 0x00, 0x00,        // reserved
    0x00, 0x00, 0x00, 0x00,  // duration = 0 -> indefinite (until a wake source / RESET_N)
    0x06, 0x00, 0x00, 0x00,  // flags: bit1 backup | bit2 force
    0x00, 0x00, 0x00, 0x00   // wakeupSources (set below)
  };
#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_EXTINT_WAKE
  pmreq[12] = 0x20;          // wakeupSources bit5 = EXTINT0 — host-controlled hot-start wake
#endif
  uint8_t ack = gnssSendUbx(0x02, 0x41, pmreq, sizeof(pmreq));
  delay(50);
  Wire.end();
  i2c_bus_initialized = false;
  // Leave bus quiet: hi-Z, no pulls — do not drive SDA/SCL after this.
  highz_no_pulls(SIOD_GPIO_NUM);
  highz_no_pulls(SIOC_GPIO_NUM);
#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_EXTINT_WAKE
  // Latch the rewired EXTINT LOW (idle) so nothing wakes the receiver until we pulse it.
  gnssExtintIdleAndHold();
  Serial.printf("[GNSS] EXTINT0 armed as wake source; GPIO%d held LOW for standby\n", GNSS_WAKE_PIN);
#endif
  // ack==0 => GNSS ACKed the PMREQ (command landed). Non-zero => the write was
  // NACKed (bus contention / back-powered peripherals) and the chip never got the
  // request — it would then stay in ~23 mA tracking through sleep.
  Serial.printf("[GNSS] UBX-RXM-PMREQ backup+force sent (I2C ack=%u, 0=OK)\n", ack);
}

#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_SLEEP_DIAG
/** Probe an I2C address and return the Wire ack (0=ACK/OK, 5=timeout i.e. line clamped). */
static uint8_t gnssDiagAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true);
}
#endif

static bool gnssReadLatestFixSinceWake(GnssFix& outFix, unsigned long timeoutMs,
                                       GnssProgressCb progressCb = nullptr) {
  outFix = GnssFix{};

  Wire.beginTransmission(GNSS_I2C_ADDR);
  if (Wire.endTransmission(true) != 0) {
    // No ACK — the module is likely in software backup. Nudge it and retry once.
    gnssWakeFromBackup();
    Wire.beginTransmission(GNSS_I2C_ADDR);
    if (Wire.endTransmission(true) != 0) {
      Serial.println("[GNSS] No ACK on I2C (0x42)");
      return false;
    }
  }

  // The module is now on permanent 3.3V, so it keeps tracking across ESP deep
  // sleep (hot start) — on wake it usually already has a fix. The DDC output
  // buffer, however, holds stale NMEA queued during sleep, so drain it tightly
  // (bounded, no per-read delay) to make sure we parse only current sentences.
  unsigned long drainStart = millis();
  while (millis() - drainStart < GNSS_DRAIN_MAX_MS) {
    uint16_t avail = 0;
    uint8_t dump[96];
    size_t got = gnssDdcReadPayload(dump, sizeof(dump), &avail);
    if (avail == 0 || got == 0) break;
  }

  String lineBuf;
  lineBuf.reserve(160);

  bool haveFix = false;
  unsigned long fixAtMs = 0;

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (progressCb) progressCb(millis() - start, timeoutMs);

    // Return as soon as we have a position. Linger only briefly (grace window)
    // to pick up a GGA for sat count / HDOP, so we don't burn a whole extra
    // NMEA epoch waiting for both RMC and GGA on an already-hot module.
    if (haveFix) {
      if (outFix.sats_used > 0 || (millis() - fixAtMs) >= GNSS_META_GRACE_MS) {
        Serial.printf("[GNSS] fix in %lums (sats=%d hot-start)\n",
                      (unsigned long)(millis() - start), outFix.sats_used);
        return true;
      }
    }

    uint16_t avail = 0;
    uint8_t chunk[96];
    size_t got = gnssDdcReadPayload(chunk, sizeof(chunk), &avail);
    if (avail == 0 || got == 0) {
      delay(20);
      continue;
    }

    for (size_t i = 0; i < got; i++) {
      char c = (char)chunk[i];
      if (c == '\r') continue;
      if (c == '\n') {
        String line = lineBuf;
        lineBuf = "";
        line.trim();
        if (line.length() == 0) continue;
        if (line[0] != '$') continue;
        if (!nmeaChecksumOK(line)) continue;

        int star = line.indexOf('*');
        String noCk = (star >= 0) ? line.substring(1, star) : line.substring(1);

        int comma = noCk.indexOf(',');
        if (comma < 0) continue;
        String type = noCk.substring(0, comma);

        if (type.endsWith("RMC")) {
          gnssApplyRMC(noCk, outFix);
        } else if (type.endsWith("GGA")) {
          gnssApplyGGA(noCk, outFix);
        }

        if (outFix.valid && !haveFix) {
          haveFix = true;
          fixAtMs = millis();
          // If this fix already carries sat count (came via GGA), done now.
          if (outFix.sats_used > 0) {
            Serial.printf("[GNSS] fix in %lums (sats=%d hot-start)\n",
                          (unsigned long)(millis() - start), outFix.sats_used);
            return true;
          }
        }
      } else {
        if (lineBuf.length() < 200) lineBuf += c;
      }
    }
  }

  return outFix.valid;
}

// Mutex so Serial output is not interleaved (Serial is not thread-safe)
static SemaphoreHandle_t s_serialMutex = NULL;

static camera_config_t s_camera_config;
static bool s_camera_initialised = false;

static void forceStopXclk(void) {
  // Best-effort: stop XCLK generation so the camera can't be partially back-powered
  // through the XCLK pin when its rail is turned off.
  // ledc_stop() can log "LEDC is not initialized" if the driver wasn't active; ignore.
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledcDetachPin(XCLK_GPIO_NUM);
  pinMode(XCLK_GPIO_NUM, OUTPUT);
  digitalWrite(XCLK_GPIO_NUM, LOW);
  delayMicroseconds(50);
  pinMode(XCLK_GPIO_NUM, INPUT);
}

static void forceStartXclk(uint32_t freq_hz = 20000000) {
  // Some camera modules won't ACK SCCB/I2C unless XCLK is running.
  // Starting XCLK explicitly makes wake-from-deep-sleep far more reliable.
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT; // 1-bit => 50% duty
  timer_conf.timer_num = LEDC_TIMER_0;
  timer_conf.freq_hz = freq_hz;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  (void)ledc_timer_config(&timer_conf);

  ledc_channel_config_t ch_conf = {};
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.gpio_num = XCLK_GPIO_NUM;
  ch_conf.duty = 1; // 50%
  ch_conf.hpoint = 0;
  (void)ledc_channel_config(&ch_conf);
}

// Phase 1b: camera init with warm-up (no EI, no lib/deploy)
static bool camera_init(void) {
  if (s_camera_initialised) return true;

  s_camera_config.ledc_channel = LEDC_CHANNEL_0;
  s_camera_config.ledc_timer = LEDC_TIMER_0;
  s_camera_config.pin_d0 = Y2_GPIO_NUM;
  s_camera_config.pin_d1 = Y3_GPIO_NUM;
  s_camera_config.pin_d2 = Y4_GPIO_NUM;
  s_camera_config.pin_d3 = Y5_GPIO_NUM;
  s_camera_config.pin_d4 = Y6_GPIO_NUM;
  s_camera_config.pin_d5 = Y7_GPIO_NUM;
  s_camera_config.pin_d6 = Y8_GPIO_NUM;
  s_camera_config.pin_d7 = Y9_GPIO_NUM;
  s_camera_config.pin_xclk = XCLK_GPIO_NUM;
  s_camera_config.pin_pclk = PCLK_GPIO_NUM;
  s_camera_config.pin_vsync = VSYNC_GPIO_NUM;
  s_camera_config.pin_href = HREF_GPIO_NUM;
  s_camera_config.pin_sccb_sda = SIOD_GPIO_NUM;
  s_camera_config.pin_sccb_scl = SIOC_GPIO_NUM;
  s_camera_config.pin_pwdn = PWDN_GPIO_NUM;
  s_camera_config.pin_reset = RESET_GPIO_NUM;
  s_camera_config.xclk_freq_hz = 20000000;
  s_camera_config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM found! Using optimized camera settings.");
    s_camera_config.frame_size = FRAMESIZE_SVGA;
    s_camera_config.jpeg_quality = 10;
    s_camera_config.fb_count = 2;
    s_camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    s_camera_config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("No PSRAM found. Using lower settings.");
    s_camera_config.frame_size = FRAMESIZE_QVGA;
    s_camera_config.jpeg_quality = 12;
    s_camera_config.fb_count = 1;
  }

  // Reset LEDC to ensure a clean XCLK start after deep sleep.
  periph_module_reset(PERIPH_LEDC_MODULE);

  // Reset the camera capture peripheral (LCD_CAM) and its DMA. A warm reset
  // (esp_restart, USB-CDC reset) only resets the CPU, leaving LCD_CAM/GDMA with
  // stale state from the previous run -> esp_camera_init succeeds but streams
  // 0 frames. Deep-sleep wake (digital core powered down) and cold boot (POR)
  // reset these for free, which is why only those worked. Harmless when already
  // in reset state. (These are enum constants on the S3, so no #ifdef guard.)
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_GDMA_MODULE);

  // Start XCLK *before* SCCB probing / esp_camera_init.
  // Many OV sensors need the clock running to respond on SCCB.
  forceStartXclk((uint32_t)s_camera_config.xclk_freq_hz);
  delay(5);

#if RESET_GPIO_NUM >= 0
  // Explicit, logged hardware reset of the sensor (active-low) with XCLK already
  // running — OV sensors need the clock during reset. Done in our own code so we
  // can confirm GPIO38 is actually driven, independent of the driver's handling.
  pinMode(RESET_GPIO_NUM, OUTPUT);
  digitalWrite(RESET_GPIO_NUM, LOW);    // assert reset
  delay(20);
  digitalWrite(RESET_GPIO_NUM, HIGH);   // release
  delay(30);                            // PLL / power-up settle
  Serial.printf("[CAM] explicit RESET pulse on GPIO%d (low 20ms, settle 30ms)\n", RESET_GPIO_NUM);
#endif

  // Ensure SCCB/I2C bus is in a sane state before probing the sensor.
  // After deep sleep wakeups the shared I2C bus can be left mid-transaction.
  i2cBusRecover();

  // #region agent log — I2C scan before esp_camera_init
  {
    Wire.end();
    delay(5);
    pinMode(SIOD_GPIO_NUM, INPUT_PULLUP);
    pinMode(SIOC_GPIO_NUM, INPUT_PULLUP);
    delay(2);
    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    Wire.setClock(100000);
    Wire.setTimeOut(50);
    Serial.printf("[DBG-59d7ad] I2C scan SDA=%d SCL=%d: ",
                  digitalRead(SIOD_GPIO_NUM), digitalRead(SIOC_GPIO_NUM));
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
      Wire.beginTransmission(addr);
      uint8_t e = Wire.endTransmission(true);
      if (e == 0) Serial.printf("0x%02X(ACK) ", addr);
      else if (addr == 0x30) Serial.printf("0x30(err=%u) ", e);
    }
    Serial.println();
    Wire.end();
    delay(2);
  }
  // #endregion

  Serial.println("Initializing camera...");
  esp_err_t err = esp_camera_init(&s_camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  Serial.println("Camera initialized successfully!");

  camera_sensor_t* s = esp_camera_sensor_get();
  if (s) {
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
      s->set_saturation(s, 0);
    }
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_special_effect(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);
  }

  // Warm-up for AEC
  Serial.printf("Warming up camera (%d frames, %dms timeout)...\n", WARMUP_TARGET_FRAMES, WARMUP_TIMEOUT_MS);
  unsigned long warmupStart = millis();
  int successfulFrames = 0;
  int nullFrames = 0;
  int consecutiveNulls = 0;
  bool warmupSuccess = true;

  for (int i = 0; i < WARMUP_TARGET_FRAMES; i++) {
    if (millis() - warmupStart > (unsigned long)WARMUP_TIMEOUT_MS) {
      Serial.printf("WARMUP TIMEOUT after %d frames\n", successfulFrames);
      warmupSuccess = false;
      break;
    }
    if (consecutiveNulls >= MAX_CONSECUTIVE_NULL_FRAMES) {
      Serial.printf("WARMUP FAIL: %d consecutive null frames\n", consecutiveNulls);
      warmupSuccess = false;
      break;
    }
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
      successfulFrames++;
      consecutiveNulls = 0;
      if (successfulFrames % 10 == 0) {
        Serial.printf("AEC warm-up: %d/%d (nulls: %d)\n", successfulFrames, WARMUP_TARGET_FRAMES, nullFrames);
      }
    } else {
      nullFrames++;
      consecutiveNulls++;
    }
    delay(WARMUP_FRAME_DELAY_MS);
  }

  Serial.printf("Warm-up done: %d/%d frames in %lums\n", successfulFrames, WARMUP_TARGET_FRAMES, millis() - warmupStart);
  if (!warmupSuccess) {
    s_camera_initialised = false;
    is_initialised = false;
    return false;
  }
  s_camera_initialised = true;
  is_initialised = true;
  return true;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif

static bool crop_top_portion_rgb888(uint8_t* input_buf, uint32_t input_width, uint32_t input_height,
                                    uint8_t* output_buf, int crop_percent) {
  if (input_buf == nullptr || output_buf == nullptr) return false;
  if (crop_percent <= 0 || crop_percent >= 100) {
    memcpy(output_buf, input_buf, input_width * input_height * 3);
    return true;
  }
  uint32_t crop_pixels = (input_height * (uint32_t)crop_percent) / 100;
  uint32_t new_height = input_height - crop_pixels;
  if (new_height == 0) return false;
  size_t row_bytes = (size_t)input_width * 3;
  size_t bytes_to_copy = (size_t)new_height * row_bytes;
  uint8_t* src_ptr = input_buf + ((size_t)crop_pixels * row_bytes);
  memmove(output_buf, src_ptr, bytes_to_copy);
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr) {
  if (snapshot_buf == nullptr || out_ptr == nullptr) return -1;
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;
  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (float)((snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix]);
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf) {
  bool do_resize = false;
  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }
  if (out_buf == nullptr) return false;
  snapshot_buf = out_buf;  // ei_camera_get_data reads from global snapshot_buf
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }
  if (classificationPhoto != nullptr) {
    heap_caps_free(classificationPhoto);
    classificationPhoto = nullptr;
  }
  classificationPhotoSize = fb->len;
  oledStatus.imageWidth = fb->width;
  oledStatus.imageHeight = fb->height;
  classificationPhoto = (uint8_t*)heap_caps_malloc(classificationPhotoSize, MALLOC_CAP_SPIRAM);
  if (classificationPhoto != nullptr) {
    memcpy(classificationPhoto, fb->buf, classificationPhotoSize);
  } else {
    classificationPhotoSize = 0;
  }
  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);
  esp_camera_fb_return(fb);
  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }
  if (topCropPercent > 0) {
    if (!crop_top_portion_rgb888(out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS,
                                 EI_CAMERA_RAW_FRAME_BUFFER_ROWS, out_buf, topCropPercent)) {
      return false;
    }
    uint32_t crop_pixels = (EI_CAMERA_RAW_FRAME_BUFFER_ROWS * (uint32_t)topCropPercent) / 100;
    uint32_t new_height = EI_CAMERA_RAW_FRAME_BUFFER_ROWS - crop_pixels;
    if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || img_height != new_height) {
      do_resize = true;
    }
  } else {
    if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS || img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
      do_resize = true;
    }
  }
  if (do_resize) {
    uint32_t actual_height = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    if (topCropPercent > 0) {
      uint32_t crop_pixels = (EI_CAMERA_RAW_FRAME_BUFFER_ROWS * (uint32_t)topCropPercent) / 100;
      actual_height = EI_CAMERA_RAW_FRAME_BUFFER_ROWS - crop_pixels;
    }
    ei::image::processing::crop_and_interpolate_rgb888(
        out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, actual_height,
        out_buf, img_width, img_height);
  }
  return true;
}

static int get_signal_data_785968(size_t offset, size_t length, float* out_ptr) {
  return ei_camera_get_data(offset, length, out_ptr);
}
static int get_signal_data_785971(size_t offset, size_t length, float* out_ptr) {
  return ei_camera_get_data(offset, length, out_ptr);
}

// Phase 2b: camera deinit (called from enterSleepMode)
void ei_camera_deinit(void) {
  if (!s_camera_initialised && !is_initialised) {
    return;  // avoid deinit warnings when camera never fully came up
  }

  // Stop XCLK first to avoid keeping the sensor partially alive via the clock pin.
  forceStopXclk();

  // Best-effort: deinit camera driver. Some Arduino/esp32-camera builds emit a gdma_disconnect warning
  // if the peripheral channel is already disconnected; it's usually harmless.
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ei_printf("Camera deinit failed (0x%x)\n", err);
  }

  is_initialised = false;
  s_camera_initialised = false;
}

static void configurePSM(void);
static void prepareModuleForPSM(void);
static void wakeModuleFromPSM(void);
static String sendATCommand(const String& cmd, unsigned long waitMs);
static bool pollSettingsOverTCP(void);
static bool runOtaUpdate(void);

static void highz_no_pulls(int g) {
  if (g < 0) return;
  pinMode(g, INPUT);
  gpio_pullup_dis((gpio_num_t)g);
  gpio_pulldown_dis((gpio_num_t)g);
}

static void stopI2CAndSleepOled(void) {
  // Best-effort: put OLED into low-power mode before cutting its rail
  u8g2.setPowerSave(1);

  // Stop I2C peripheral; prevents it from keeping pins/peripheral active
  Wire.end();

  // Make bus pins high-Z (note: external pull-ups to 3V3 can still back-power
  // unpowered peripherals; hardware fix is pull-ups to switched rail or bus isolation)
  highz_no_pulls(SIOD_GPIO_NUM);
  highz_no_pulls(SIOC_GPIO_NUM);
}

/** Drive a GPIO LOW and enable deep-sleep hold so an unpowered peripheral cannot back-feed. */
static void holdPinLowForSleep(int g) {
  if (g < 0) return;
  pinMode(g, OUTPUT);
  digitalWrite(g, LOW);
  if (rtc_gpio_is_valid_gpio((gpio_num_t)g)) {
    rtc_gpio_hold_en((gpio_num_t)g);
  } else {
    gpio_hold_en((gpio_num_t)g);
  }
}

static void prepareVddDomainPinsForSleep(void) {
  // Pins connected to peripherals powered by LD_SWCH_1/LD_SWCH_2 should not be driving
  // when the load switches are turned off. Put them in high-Z with no pulls.
  const int vddPins[] = {
      // Camera SCCB/I2C
      SIOD_GPIO_NUM,
      SIOC_GPIO_NUM,
      // Camera timing
      VSYNC_GPIO_NUM,
      HREF_GPIO_NUM,
      XCLK_GPIO_NUM,
      PCLK_GPIO_NUM,
      // Camera reset line (hi-Z while rail is off; no back-feed into the sensor)
      RESET_GPIO_NUM,
      // Camera data bus
      Y2_GPIO_NUM,
      Y3_GPIO_NUM,
      Y4_GPIO_NUM,
      Y5_GPIO_NUM,
      Y6_GPIO_NUM,
      Y7_GPIO_NUM,
      Y8_GPIO_NUM,
      Y9_GPIO_NUM,
  };
  for (size_t i = 0; i < sizeof(vddPins) / sizeof(vddPins[0]); i++) {
    highz_no_pulls(vddPins[i]);
  }
}

static void enterSleepMode(void);

static int clampSleepSeconds(int sec);

#if PIR_WAKE_ENABLE
static bool s_enablePirWakeOnNextSleep = false;
static bool s_pirExt1Armed = false;

static void setPirWakeOnNextSleep(bool enable) { s_enablePirWakeOnNextSleep = enable; }

static bool isMotionExtWakeCause(esp_sleep_wakeup_cause_t cause) {
  return cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_EXT1;
}

static bool pirInputIsHigh(void) { return digitalRead(PIR_WAKE_GPIO) == HIGH; }

static void configurePirRtcInputPullDown(gpio_num_t pirPin) {
  rtc_gpio_init(pirPin);
  rtc_gpio_set_direction(pirPin, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(pirPin);
  rtc_gpio_pulldown_en(pirPin);
}

/** OUT must read LOW continuously for stableMs (poll every 10 ms). */
static bool waitPirLineLowStable(unsigned long stableMs, unsigned long timeoutMs) {
  unsigned long stableStart = 0;
  const unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    if (pirInputIsHigh()) {
      stableStart = 0;
    } else if (stableStart == 0) {
      stableStart = millis();
    } else if (millis() - stableStart >= stableMs) {
      return true;
    }
    delay(10);
  }
  return false;
}

/** OUT must read HIGH continuously for stableMs. */
static bool waitPirLineHighStable(unsigned long stableMs, unsigned long timeoutMs) {
  unsigned long stableStart = 0;
  const unsigned long deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    if (!pirInputIsHigh()) {
      stableStart = 0;
    } else if (stableStart == 0) {
      stableStart = millis();
    } else if (millis() - stableStart >= stableMs) {
      return true;
    }
    delay(10);
  }
  return false;
}

static void disarmPirExt1Wake(void) {
  if (!s_pirExt1Armed) return;
  if (esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1) == ESP_OK) {
    s_pirExt1Armed = false;
  }
}

/** Arm EXT1 only when OUT is quiet LOW. Call immediately before esp_deep_sleep_start(). */
static void preparePirWakeForDeepSleep(bool enable) {
  disarmPirExt1Wake();
  if (!enable) {
    Serial.println("[PIR] Motion wake disabled for this sleep");
    return;
  }
  const gpio_num_t pirPin = (gpio_num_t)PIR_WAKE_GPIO;
  if (!rtc_gpio_is_valid_gpio(pirPin)) {
    Serial.printf("[PIR] GPIO%d is not RTC-capable — motion wake skipped\n", PIR_WAKE_GPIO);
    return;
  }

  configurePirRtcInputPullDown(pirPin);
  delay(PIR_WAKE_ARM_QUIET_MS);

  const int level = digitalRead(PIR_WAKE_GPIO);
  Serial.printf("[PIR] GPIO%d level before arm: %d\n", PIR_WAKE_GPIO, level);

  if (!waitPirLineLowStable(PIR_WAKE_SETTLE_MS, 3000)) {
    Serial.println("[PIR] OUT not stable LOW — timer-only sleep (skip EXT1)");
    return;
  }

  esp_err_t err = esp_sleep_enable_ext1_wakeup(1ULL << PIR_WAKE_GPIO, ESP_EXT1_WAKEUP_ANY_HIGH);
  if (err != ESP_OK) {
    Serial.printf("[PIR] Failed to enable EXT1 wake: %s\n", esp_err_to_name(err));
    return;
  }
  s_pirExt1Armed = true;

  delay(20);
  if (pirInputIsHigh()) {
    Serial.println("[PIR] OUT HIGH right after arming — disarming EXT1 (entry glitch)");
    disarmPirExt1Wake();
    return;
  }

  Serial.printf("[PIR] EXT1 wake armed on GPIO%d (HIGH)\n", PIR_WAKE_GPIO);
}

/** False EXT1 wake (glitch during sleep entry): re-enter deep sleep with timer only. */
static void handleSpuriousPirWakeAndResleep(void) {
  const gpio_num_t pirPin = (gpio_num_t)PIR_WAKE_GPIO;
  if (rtc_gpio_is_valid_gpio(pirPin)) {
    configurePirRtcInputPullDown(pirPin);
  } else {
    pinMode(PIR_WAKE_GPIO, INPUT);
    gpio_pullup_dis(pirPin);
    gpio_pulldown_en(pirPin);
  }
  delay(PIR_WAKE_ARM_QUIET_MS);

  if (waitPirLineHighStable(PIR_WAKE_CONFIRM_MS, 500)) {
    Serial.println("[PIR] Motion confirmed after EXT wake");
    return;
  }

  disarmPirExt1Wake();
  const int sleepSec = clampSleepSeconds(customSleepDuration > 0 ? customSleepDuration : DEFAULT_SLEEP_TIME);
  Serial.printf("[PIR] Spurious EXT wake (OUT not stable HIGH) — timer-only sleep %ds\n", sleepSec);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}
#endif  // PIR_WAKE_ENABLE

/** Daytime deep sleep: timer + optional PIR (operational window). PIR arming
 *  follows the per-state mode captured in s_daytimePirArm. */
static void enterDaytimeSleepMode(void) {
#if PIR_WAKE_ENABLE
  setPirWakeOnNextSleep(s_daytimePirArm);
#endif
  enterSleepMode();
}

/** Night schedule sleep: timer only until next WAKE (no PIR). */
static void enterNightScheduleSleepMode(void) {
#if PIR_WAKE_ENABLE
  setPirWakeOnNextSleep(false);
#else
  (void)0;
#endif
  enterSleepMode();
}

// Phase 2b: full enterSleepMode (PSM optional; timer + VDD off)
static void enterSleepMode(void) {
  Serial.println("Preparing for sleep...");

  // Some paths reach here with the I2C bus torn down (e.g. early no-fix/home
  // sleep does Wire.end() before sleeping). PSM config and the sleep message
  // below both draw to the OLED, so restore the bus first to avoid NULL TX
  // buffer errors. No-op if the bus is already up (normal path).
  if (s_oled_initialized) {
    initI2CBusOnce();
  }

#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_SLEEP_MODEM_OFF_TEST
  Serial.println("[GNSS-DIAG] MODEM_OFF: AT+CFUN=0 (removing modem RF as a wake variable)");
  sendATCommand("AT+CFUN=0", 2000);
  delay(500);
#else
  if (usePSM) {
    Serial.println("Configuring NB-IoT PSM...");
    configurePSM();
    prepareModuleForPSM();
  }
#endif

  // 1) Stop radios
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  // 2) Deinit camera
  if (s_camera_initialised || is_initialised) {
    Serial.println("Deinitializing camera...");
    ei_camera_deinit();
    delay(100);
  }

  // 3) Sleep timer (+ optional PIR during daytime)
  esp_sleep_enable_timer_wakeup((uint64_t)customSleepDuration * uS_TO_S_FACTOR);
  Serial.printf("Setup ESP32 to sleep for %d seconds\n", customSleepDuration);

  // 4) OLED sleep message (while still powered)
  char sleepMsg[40];
  if (s_sleepOledExtra[0] != '\0') {
    snprintf(sleepMsg, sizeof(sleepMsg), "%ds %s", customSleepDuration, s_sleepOledExtra);
  } else {
    snprintf(sleepMsg, sizeof(sleepMsg), "Sleep %ds", customSleepDuration);
  }
  if (s_oled_initialized) {
    oled_status("SLEEPING...", sleepMsg, -1);
    // 5) Put OLED to sleep + stop I2C before powering down its rail
    stopI2CAndSleepOled();
  } else {
    Serial.printf("SLEEPING... %s (no OLED this wake)\n", sleepMsg);
  }

  // 6) Peripheral pins benign before cutting switched rails.
  prepareVddDomainPinsForSleep();
  pinMode(XCLK_GPIO_NUM, OUTPUT);
  digitalWrite(XCLK_GPIO_NUM, LOW);
  highz_no_pulls(BC95_RX_PIN);
  highz_no_pulls(BC95_TX_PIN);

#if HAS_GNSS && GNSS_POWER_PERMANENT
  // 7) MAX-M10S on permanent 3.3 V, sharing this single DDC/I2C bus (pull-ups to the
  // permanent 3V3 rail) with the camera (LD_SWCH_1) and the OLED + MAX17048 (LD_SWCH_2).
  // Two hard constraints collide:
  //   * the backup request (UBX-RXM-PMREQ) must be the LAST bus edge — the M10 wakes
  //     from software backup on any DDC activity, so cutting a bus peripheral's rail
  //     AFTER backup re-wakes it into ~23 mA tracking; AND
  //   * the write needs a live bus — once a peripheral loses power its I/O pins clamp
  //     SDA/SCL low and the master times out (observed: I2C ack=5), so the request
  //     never lands and the M10 also stays awake.
  // Resolution: send the backup request while the camera is STILL powered (the bus is
  // healthy → ack 0), THEN cut the camera rail with no further I2C. Sequencing it the
  // other way (cut rail first, then PMREQ over the clamped bus) is what produced ack=5
  // and left the M10 tracking. LD_SWCH_2 stays powered through sleep so that only the
  // camera can clamp the bus on rail-cut (costs OLED/MAX17048 idle current, far less than
  // the GNSS tracking current it saves).
  //
  // Remaining risk: cutting the camera rail clamps the shared SDA/SCL low — a falling edge
  // the M10 may read as DDC activity and wake on. If measured sleep current stays ~24 mA
  // even though ack reads 0, that glitch is the cause and hardware bus isolation is
  // required (GNSS on its own I2C bus, or a bus switch/isolator on the camera SCCB). Build
  // with -DGNSS_SLEEP_DIAG=1 to confirm the clamp (logs ack before/after the rail cut).
  initI2CBusOnce();
#if GNSS_SLEEP_DIAG
  Serial.printf("[GNSS-DIAG] rails ON: ack 0x42=%u 0x3C=%u 0x36=%u (expect 0,0,0)\n",
                gnssDiagAck(GNSS_I2C_ADDR), gnssDiagAck(0x3C), gnssDiagAck(MAX17048_I2C_ADDR));
#endif
  gnssEnterBackup();  // PMREQ on a healthy bus → ack should be 0; M10 enters software backup
#if GNSS_SLEEP_NO_RAIL_CUT
  Serial.println("[GNSS-DIAG] NO_RAIL_CUT: leaving camera rail (LD_SWCH_1) POWERED through sleep");
#else
  Serial.println("Turning camera rail OFF (LD_SWCH_1); keeping LD_SWCH_2 alive for I2C");
  digitalWrite(LD_SWCH_1, LOW);
  delay(20);
#endif
#if GNSS_SLEEP_DIAG
  // Characterise the clamp. This re-touches the bus and WILL wake the M10 — diag builds only.
  initI2CBusOnce();
  Serial.printf("[GNSS-DIAG] camera OFF: ack 0x42=%u (5=clamped/timeout confirms H1)\n",
                gnssDiagAck(GNSS_I2C_ADDR));
  Wire.end();
  i2c_bus_initialized = false;
  highz_no_pulls(SIOD_GPIO_NUM);
  highz_no_pulls(SIOC_GPIO_NUM);
#endif
#else
  // GNSS on LD_SWCH_1 (or no GNSS): sink shared I2C before rail off. If SDA/SCL
  // are left floating while the camera is unpowered, the sensor back-feeds ~20 mA
  // through the shared bus pull-ups.
  pinMode(SIOC_GPIO_NUM, OUTPUT);
  digitalWrite(SIOC_GPIO_NUM, LOW);
  pinMode(SIOD_GPIO_NUM, OUTPUT);
  digitalWrite(SIOD_GPIO_NUM, LOW);

  // 7) Power down switchable peripherals before deep sleep
  // LD_SWCH_1: camera (+ GNSS if jumper VDD1). LD_SWCH_2: OLED, MAX17048.
  Serial.println("Turning load switches OFF");
  digitalWrite(LD_SWCH_1, LOW);
  digitalWrite(LD_SWCH_2, LOW);
  delay(5);

  holdPinLowForSleep(SIOC_GPIO_NUM);
  holdPinLowForSleep(SIOD_GPIO_NUM);
#endif

  // Hold load switches + all camera-exclusive lines LOW. Unpowered camera modules
  // can otherwise back-feed ~20–30 mA through GPIO data pins (non-RTC pins cannot
  // be rtc_gpio_isolate'd on ESP32-S3).
#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_SLEEP_NO_RAIL_CUT
  // Diagnostic: camera stays powered, so hold LD_SWCH_1 HIGH and DO NOT drive the camera
  // DVP/SCCB pins (the sensor may be driving them) — leave them hi-Z (set earlier).
  pinMode(LD_SWCH_1, OUTPUT);
  digitalWrite(LD_SWCH_1, HIGH);
  if (rtc_gpio_is_valid_gpio((gpio_num_t)LD_SWCH_1)) rtc_gpio_hold_en((gpio_num_t)LD_SWCH_1);
  else gpio_hold_en((gpio_num_t)LD_SWCH_1);
#else
  holdPinLowForSleep(LD_SWCH_1);
#endif
#if HAS_GNSS && GNSS_POWER_PERMANENT
  // Keep the OLED/MAX17048 rail powered through sleep so the shared I2C bus stays
  // quiescent-high. Cutting it would clamp SDA/SCL and wake the M10 from backup.
  pinMode(LD_SWCH_2, OUTPUT);
  digitalWrite(LD_SWCH_2, HIGH);
  if (rtc_gpio_is_valid_gpio((gpio_num_t)LD_SWCH_2)) rtc_gpio_hold_en((gpio_num_t)LD_SWCH_2);
  else gpio_hold_en((gpio_num_t)LD_SWCH_2);
#else
  holdPinLowForSleep(LD_SWCH_2);
#endif
#if !(HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_SLEEP_NO_RAIL_CUT)
  // Camera is unpowered: drive its lines LOW to stop back-feed. In NO_RAIL_CUT diag the
  // camera is still powered, so skip these (driving its outputs would cause contention).
  holdPinLowForSleep(XCLK_GPIO_NUM);
  holdPinLowForSleep(PCLK_GPIO_NUM);
  holdPinLowForSleep(VSYNC_GPIO_NUM);
  holdPinLowForSleep(HREF_GPIO_NUM);
  holdPinLowForSleep(Y2_GPIO_NUM);
  holdPinLowForSleep(Y3_GPIO_NUM);
  holdPinLowForSleep(Y4_GPIO_NUM);
  holdPinLowForSleep(Y5_GPIO_NUM);
  holdPinLowForSleep(Y6_GPIO_NUM);
  holdPinLowForSleep(Y7_GPIO_NUM);
  holdPinLowForSleep(Y8_GPIO_NUM);
  holdPinLowForSleep(Y9_GPIO_NUM);
#endif
  gpio_deep_sleep_hold_en();

  // 8) Free classification photo
  if (classificationPhoto != nullptr) {
    heap_caps_free(classificationPhoto);
    classificationPhoto = nullptr;
    classificationPhotoSize = 0;
    Serial.println("Cleaned up classification photo");
  }

  // 9) Power-saving:
  // Keep RTC peripherals ON so RTC GPIO hold (load switch pins) remains effective.
  // This costs a bit of sleep current but prevents peripherals from being partially powered.
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);

  // 10) Optional: isolate GPIOs to prevent leakage (RTC IOs only)
  // Note: rtc_gpio_isolate() only works on RTC-capable IOs. On ESP32-S3 many GPIOs are NOT RTC IOs.
  // Calling it on non-RTC pins spams logs: "RTCIO number error".
  // Do not rtc_gpio_isolate SDA/SCL when GNSS stays powered — isolation glitches the bus.
  const int gpiosToIsolate[] = {
#if !HAS_GNSS
      SIOD_GPIO_NUM, SIOC_GPIO_NUM,
#endif
      VSYNC_GPIO_NUM, HREF_GPIO_NUM, XCLK_GPIO_NUM, PCLK_GPIO_NUM, Y2_GPIO_NUM, Y3_GPIO_NUM,
      Y4_GPIO_NUM, Y5_GPIO_NUM, Y6_GPIO_NUM, Y7_GPIO_NUM, Y8_GPIO_NUM, Y9_GPIO_NUM,
      BC95_RX_PIN, BC95_TX_PIN};
  for (size_t i = 0; i < sizeof(gpiosToIsolate) / sizeof(gpiosToIsolate[0]); i++) {
    int g = gpiosToIsolate[i];
    if (g < 0) continue;
    if (!rtc_gpio_is_valid_gpio((gpio_num_t)g)) continue;
    rtc_gpio_isolate((gpio_num_t)g);
  }

  delay(100);
#if PIR_WAKE_ENABLE
  {
#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_SLEEP_MODEM_OFF_TEST
    const bool wantPir = false;  // diag: timer-only sleep, remove PIR/EXT1 as a variable
#else
    const bool wantPir = s_enablePirWakeOnNextSleep;
#endif
    s_enablePirWakeOnNextSleep = false;
    preparePirWakeForDeepSleep(wantPir);
  }
#endif
  Serial.println("Going to sleep now");
  Serial.flush();
#if CAM_DEBUG_RESTART_INSTEAD_OF_SLEEP
  // DIAGNOSTIC: reboot instead of deep sleep. If the camera streams on every
  // cycle now (vs. failing after a real deep-sleep wake), the fault is the
  // deep-sleep SoC state, not the camera hardware. Revert the flag after.
  Serial.println("[CAM-DEBUG] esp_restart() instead of deep sleep");
  Serial.flush();
  delay(200);
  esp_restart();
#else
  esp_deep_sleep_start();
#endif
}

// Phase 2b: BC95 AT command layer
static void modemFlushInput(unsigned long drainMs = 0) {
  unsigned long start = millis();
  while (bc95_modem.available() || (drainMs > 0 && (millis() - start) < drainMs)) {
    while (bc95_modem.available()) {
      (void)bc95_modem.read();
    }
    if (drainMs == 0) break;
  }
}

static int waitForAnyOf(const char* token1, const char* token2, unsigned long timeoutMs, String& out) {
  out = "";
  unsigned long start = millis();
  delay(10);
  while (millis() - start < timeoutMs) {
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      out += c;
      if (token1 && strstr(out.c_str(), token1)) return 1;
      if (token2 && strstr(out.c_str(), token2)) return 2;
    }
    delay(10);
  }
  return 0;
}

static bool waitForQIOPEN(int connectID, unsigned long timeoutMs, const String& existingResponse) {
  String qiopenPattern = String("+QIOPEN: ") + String(connectID) + ",";
  if (existingResponse.length() > 0 && existingResponse.indexOf(qiopenPattern) != -1) {
    int start = existingResponse.indexOf(qiopenPattern);
    String resultPart = existingResponse.substring(start + qiopenPattern.length());
    int resultCode = resultPart.toInt();
    return (resultCode == 0);
  }
  String resp;
  int which = waitForAnyOf(qiopenPattern.c_str(), nullptr, timeoutMs, resp);
  if (which == 1) {
    int start = resp.indexOf(qiopenPattern);
    if (start != -1) {
      String resultPart = resp.substring(start + qiopenPattern.length());
      int resultCode = resultPart.toInt();
      return (resultCode == 0);
    }
  }
  return false;
}

static bool waitForSendResult(unsigned long timeoutMs, const String& existingResponse) {
  if (existingResponse.length() > 0) {
    if (existingResponse.indexOf("SEND OK") != -1) return true;
    if (existingResponse.indexOf("SEND FAIL") != -1) return false;
  }
  String resp;
  int which = waitForAnyOf("SEND OK", "SEND FAIL", timeoutMs, resp);
  if (which == 1) return true;
  if (which == 2) return false;
  return false;
}

// Binary send via AT+QISEND (prompted). Halves airtime vs hex-encoded AT+QISENDEX
// because each payload byte is not hex-expanded on the UART and on the air.
// Flow: "AT+QISEND=<id>,<len>" -> wait for '>' prompt -> stream raw bytes ->
// wait for "SEND OK" / "SEND FAIL" / "ERROR".
// Query the modem's per-connection sent-byte counter: AT+QISEND=<id>,0 replies
// "+QISEND: <total_send_length>,<ackedbytes>,<unackedbytes>". Returns -1 when the
// reply can't be parsed (treat as "unknown", never as zero).
static long queryTCPSentBytes(int connectID) {
  String r = sendATCommand(String("AT+QISEND=") + String(connectID) + ",0", 2000);
  int p = r.indexOf("+QISEND:");
  if (p == -1) return -1;
  p += 8;
  while (p < (int)r.length() && (r[p] == ' ')) p++;
  long total = 0;
  bool any = false;
  while (p < (int)r.length() && isdigit((unsigned char)r[p])) { total = total * 10 + (r[p] - '0'); p++; any = true; }
  return any ? total : -1;
}

static bool sendBinaryOverTCP(int connectID, const uint8_t* data, size_t length,
                              unsigned long promptTimeoutMs = 5000,
                              unsigned long ackTimeoutMs = 10000) {
  if (data == nullptr || length == 0) return false;

  // Baseline of the modem's sent counter BEFORE this send. If the SEND OK
  // acknowledgement later gets garbled on the UART (seen at 115200), we can ask
  // the modem whether it actually took the bytes instead of blindly resending —
  // a blind resend duplicates the chunk inside the TCP stream and corrupts the
  // image on the server (accumulated > declared image_size, file never written).
  long sentBefore = queryTCPSentBytes(connectID);

  modemFlushInput(5);
  String cmd = String("AT+QISEND=") + String(connectID) + "," + String((unsigned long)length) + "\r\n";
  bc95_modem.print(cmd);

  // Wait for '>' prompt.
  unsigned long start = millis();
  bool gotPrompt = false;
  String preamble;
  preamble.reserve(32);
  while (millis() - start < promptTimeoutMs) {
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      if (c == '>') { gotPrompt = true; break; }
      preamble += c;
      if (preamble.indexOf("ERROR") != -1) return false;
    }
    if (gotPrompt) break;
    delay(2);
  }
  if (!gotPrompt) return false;

  // Stream raw payload.
  size_t written = 0;
  while (written < length) {
    size_t n = bc95_modem.write(data + written, length - written);
    if (n == 0) { delay(1); continue; }
    written += n;
  }
  bc95_modem.flush();

  // Wait for acknowledgement.
  String resp;
  resp.reserve(64);
  unsigned long ackStart = millis();
  while (millis() - ackStart < ackTimeoutMs) {
    while (bc95_modem.available()) {
      resp += (char)bc95_modem.read();
      if (resp.indexOf("SEND OK") != -1) return true;
      if (resp.indexOf("SEND FAIL") != -1) return false;
      if (resp.indexOf("\r\nERROR") != -1) return false;
      if (resp.indexOf("+CME ERROR") != -1) return false;
    }
    delay(5);
  }

  // ACK timed out or arrived garbled. Before reporting failure (which makes the
  // caller RESEND the same chunk → duplicate bytes in the TCP stream), ask the
  // modem how many bytes it has actually accepted on this connection. If the
  // counter moved by exactly this chunk, the send succeeded and only the UART
  // response was lost.
  if (sentBefore >= 0) {
    long sentAfter = queryTCPSentBytes(connectID);
    if (sentAfter >= 0 && sentAfter == sentBefore + (long)length) {
      Serial.printf("[TCP] ACK garbled but modem counter confirms %u bytes accepted (%ld -> %ld) — not resending\n",
                    (unsigned)length, sentBefore, sentAfter);
      return true;
    }
    if (sentAfter >= 0) {
      Serial.printf("[TCP] ACK timeout, counter %ld -> %ld (expected %ld) — chunk NOT accepted, retry is safe\n",
                    sentBefore, sentAfter, sentBefore + (long)length);
    }
  }
  return false;
}

static String sendATCommand(const String& cmd, unsigned long waitMs = 1000) {
  modemFlushInput(5);
  bc95_modem.print(cmd + "\r\n");
  String response;
  bool seenTerminator = false;
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      response += c;
      if (response.indexOf("\r\nOK") != -1 || response.indexOf("\r\nERROR") != -1) {
        seenTerminator = true;
      }
    }
    if (seenTerminator) break;
  }
  response.trim();
  return response;
}

/** Update oledStatus.signalRSSI from AT+CSQ (0-31 or 99). Retries while the modem
 *  still reports 99 (unknown) — common right after a cold-boot (re)registration. */
static void updateOledSignalFromCSQ(void) {
  for (int attempt = 0; attempt < 5; attempt++) {
    String csqResponse = sendATCommand("AT+CSQ", 1000);
    int csqPos = csqResponse.indexOf("+CSQ:");
    if (csqPos != -1) {
      int commaPos = csqResponse.indexOf(',', csqPos);
      if (commaPos != -1) {
        String rssiStr = csqResponse.substring(csqPos + 5, commaPos);
        rssiStr.trim();
        int rssi = rssiStr.toInt();
        if (rssi != 99) {            // valid reading — keep it and stop
          oledStatus.signalRSSI = rssi;
          return;
        }
        oledStatus.signalRSSI = rssi; // remember 99 in case all attempts fail
      }
    }
    delay(600);                       // let the modem settle after registration
  }
}

// ---- Operational-window time helpers ----

struct NetworkTime {
  int year, month, day;
  int hour, minute, second;
  int tzQuarterHours;
  bool hasTz;
  bool valid;
};

/** Parse full date/time/TZ from the quoted CCLK payload "YY/MM/DD,HH:MM:SS±ZZ". */
static bool parseNetworkTimeFull(const String& payload, NetworkTime& nt) {
  nt = {};
  String s = payload;
  s.trim();
  if (s.startsWith("\"")) s = s.substring(1);
  if (s.endsWith("\""))   s = s.substring(0, s.length() - 1);
  if (s.length() < 17) return false;

  nt.year   = s.substring(0, 2).toInt();
  nt.month  = s.substring(3, 5).toInt();
  nt.day    = s.substring(6, 8).toInt();
  nt.hour   = s.substring(9, 11).toInt();
  nt.minute = s.substring(12, 14).toInt();
  nt.second = s.substring(15, 17).toInt();

  if (s.length() >= 20) {
    char sign = s.charAt(17);
    if (sign == '+' || sign == '-') {
      int tzVal = s.substring(18, 20).toInt();
      nt.tzQuarterHours = (sign == '-') ? -tzVal : tzVal;
      nt.hasTz = true;
    }
  }

  if (nt.year < 24 || nt.month < 1 || nt.month > 12 ||
      nt.day < 1 || nt.day > 31 || nt.hour > 23 || nt.minute > 59) {
    return false;
  }
  nt.valid = true;
  return true;
}

// ---- fallback timezone when CCLK carries no TZ suffix -------------------
// Day of week (0 = Sunday) for a Gregorian date — Sakamoto's method.
static int dayOfWeekYmd(int y, int m, int d) {
  static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y -= 1;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

static int lastSundayOfMonth(int year, int month) {
  static const int mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int days = mdays[month - 1];
  if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) days = 29;
  return days - dayOfWeekYmd(year, month, days);
}

/** Effective fallback TZ (quarter-hours) for a UTC timestamp. With
 *  FALLBACK_TZ_AUTO_CZ this is CET/CEST by the EU rule: summer time between
 *  the last Sunday of March 01:00 UTC and the last Sunday of October 01:00 UTC. */
static int fallbackTzQuarterHours(const NetworkTime& nt) {
#if FALLBACK_TZ_AUTO_CZ
  const int year = 2000 + nt.year;
  const int marSun = lastSundayOfMonth(year, 3);
  const int octSun = lastSundayOfMonth(year, 10);
  bool summer;
  if (nt.month > 3 && nt.month < 10)      summer = true;
  else if (nt.month < 3 || nt.month > 10) summer = false;
  else if (nt.month == 3)                 summer = (nt.day > marSun) || (nt.day == marSun && nt.hour >= 1);
  else /* October */                      summer = (nt.day < octSun) || (nt.day == octSun && nt.hour < 1);
  return summer ? 8 : 4;   // CEST = UTC+2, CET = UTC+1
#else
  (void)nt;
  return FALLBACK_TZ_QUARTER_HOURS;
#endif
}

/** Convert NetworkTime to local second-of-day (0..86399). */
static int resolveLocalSecondOfDay(const NetworkTime& nt) {
  int sec = nt.hour * 3600 + nt.minute * 60 + nt.second;
  if (!nt.hasTz) {
    sec += fallbackTzQuarterHours(nt) * 900;
    sec %= 86400;
    if (sec < 0) sec += 86400;
  }
  return sec;
}

// ---- local clock persisted across deep sleep ----------------------------
// ESP-IDF keeps the system clock advancing from the RTC across deep sleep, so a
// clock seeded once from the modem survives every timer/PIR wake; only a true
// power-on loses it. We store local-time-of-day as the epoch, so
// time(NULL) % 86400 IS the local second of day (day rollover included).
// This lets the operational-window gate run with zero radio on normal wakes.
RTC_DATA_ATTR bool g_rtcClockValid = false;
// Firmware version that seeded the persisted clock. RTC memory survives the
// esp_restart after an OTA (and USB reflash), so a clock seeded under old TZ
// semantics would otherwise outlive the firmware that wrote it — exactly how
// the v9 +2h clock would have kept the window shifted even after the v10 fix.
// A version mismatch forces one modem re-sync on the first wake.
RTC_DATA_ATTR uint32_t g_rtcClockFwVersion = 0;

static void persistLocalClock(int localSecOfDay) {
  struct timeval tv;
  tv.tv_sec = (time_t)localSecOfDay;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  g_rtcClockValid = true;
  g_rtcClockFwVersion = (uint32_t)FW_VERSION;
}

static bool rtcLocalSecondOfDay(int& outSec) {
  if (!g_rtcClockValid) return false;
  if (g_rtcClockFwVersion != (uint32_t)FW_VERSION) {
    Serial.printf("[SCHED] Persisted clock is from fw v%u (now v%u) — discarding, will re-sync from modem.\n",
                  (unsigned)g_rtcClockFwVersion, (unsigned)FW_VERSION);
    g_rtcClockValid = false;
    return false;
  }
  time_t now = time(nullptr);
  if (now <= 0) return false;
  outSec = (int)(now % 86400);
  return true;
}

// Operational window boundaries come from the NVS config (formerly the WAKE_/
// NAP_ compile-time macros, now the defaults). g_sleepGeo is loaded before any
// scheduling decision (loadSleepGeoFromNvs runs early in setup).
static int cfgWakeSec(void) { return g_sleepGeo.wake_hour * 3600 + g_sleepGeo.wake_minute * 60; }
static int cfgNapSec(void)  { return g_sleepGeo.nap_hour  * 3600 + g_sleepGeo.nap_minute  * 60; }

/** True when localSecOfDay falls inside the WAKE→NAP active window. */
static bool isInsideOperationalWindow(int localSecOfDay) {
  const int wakeSec = cfgWakeSec();
  const int napSec  = cfgNapSec();
  if (wakeSec == napSec) return true;
  if (wakeSec < napSec)
    return (localSecOfDay >= wakeSec && localSecOfDay < napSec);
  return (localSecOfDay >= wakeSec || localSecOfDay < napSec);
}

/** Seconds from localSecOfDay until the next WAKE boundary. */
static int secondsUntilNextWake(int localSecOfDay) {
  const int wakeSec = cfgWakeSec();
  if (localSecOfDay < wakeSec)
    return wakeSec - localSecOfDay;
  return 86400 - localSecOfDay + wakeSec;
}

/** Seconds from localSecOfDay until the next NAP boundary. */
static int secondsUntilNextNap(int localSecOfDay) {
  const int napSec = cfgNapSec();
  if (localSecOfDay < napSec)
    return napSec - localSecOfDay;
  return 86400 - localSecOfDay + napSec;
}

// Local time-of-day captured by the night-schedule gate each wake, used to
// compute the daytime "sleep until NAP" fallback. -1 until known this boot.
static int g_localSecOfDay = -1;
static unsigned long g_localTimeCapturedMs = 0;

// ---- Legacy HH:MM-only parser (still used for OLED display) ----

/** Parse network time from AT+CCLK response. Expected: "25/06/08,07:29:51+08" (quoted). */
static bool parseNetworkTime(const String& timeResponse, int& hour, int& minute) {
  int commaIndex = timeResponse.indexOf(',');
  if (commaIndex == -1) return false;
  String timePart = timeResponse.substring(commaIndex + 1);
  int colonIndex = timePart.indexOf(':');
  if (colonIndex == -1) return false;
  hour = timePart.substring(0, colonIndex).toInt();
  minute = timePart.substring(colonIndex + 1, colonIndex + 3).toInt();
  return true;
}

/** Query network time via AT+CCLK? and set hour/minute. */
static bool queryNetworkTime(int& hour, int& minute, String& rawOut) {
  rawOut = "";
  modemFlushInput(10);
  String resp = sendATCommand("AT+CCLK?", 1000);
  if (resp.indexOf("+CCLK:") == -1) {
    String extra;
    int which = waitForAnyOf("+CCLK:", nullptr, 2000, extra);
    if (which == 0) return false;
    resp += extra;
  }
  int idx = resp.lastIndexOf("+CCLK:");
  if (idx < 0) idx = resp.indexOf("+CCLK:");
  if (idx < 0) return false;
  int lineEnd = resp.indexOf('\n', idx);
  String line = (lineEnd > idx) ? resp.substring(idx, lineEnd) : resp.substring(idx);
  rawOut = line;
  int q1 = line.indexOf('"');
  int q2 = (q1 >= 0) ? line.indexOf('"', q1 + 1) : -1;
  String toParse = (q1 >= 0 && q2 > q1) ? line.substring(q1, q2 + 1) : line;
  return parseNetworkTime(toParse, hour, minute);
}

static bool waitForModuleBoot(int maxAttempts, int delayMs) {
  Serial.println("Waiting for NB-IoT module to boot...");
  oled_status("wait module", "boot", -1);
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    modemFlushInput(20);
    bc95_modem.print("AT\r\n");
    delay(100);
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 500) {
      if (bc95_modem.available()) {
        char c = (char)bc95_modem.read();
        response += c;
      }
    }
    if (response.indexOf("OK") != -1) {
      Serial.printf("Module responded after %d attempts\n", attempt);
      oled_status("module boot", "OK", -1);
      modemFlushInput(50);
      return true;
    }
    if (attempt % 5 == 0) {
      Serial.printf("Still waiting... attempt %d/%d\n", attempt, maxAttempts);
    }
    delay(delayMs);
  }
  Serial.println("WARNING: Module did not respond");
  oled_status("module boot", "FAIL", -1);
  return false;
}

static void configurePSM(void) {
  if (!usePSM) {
    sendATCommand("AT+CPSMS=0", 1000);
    sendATCommand("AT+QSCLK=0", 1000);
    return;
  }
  Serial.println("Configuring PSM...");
  oled_status("config PSM", "mode", -1);
  sendATCommand("AT+QNBIOTEVENT=1,1", 1000);
  if (useEDRX) {
    String edrxCmd = String("AT+CEDRXS=1,5,\"") + EDRX_CYCLE + "\"";
    sendATCommand(edrxCmd, 2000);
  } else {
    sendATCommand("AT+CEDRXS=0,5", 1000);
  }
  String psmCmd = String("AT+CPSMS=1,,,\"") + PSM_PERIODIC_TAU + "\",\"" + PSM_ACTIVE_TIME + "\"";
  sendATCommand(psmCmd, 2000);
  sendATCommand("AT+QSCLK=1", 1000);
  sendATCommand("AT&W", 500);
  sendATCommand("AT+CEREG=4", 500);
  sendATCommand("AT+CEREG?", 1000);
  oled_status("PSM config", "done", -1);
}

static void prepareModuleForPSM(void) {
  if (!usePSM) return;
  Serial.println("Preparing module for PSM...");
  oled_status("prepare PSM", "sleep", -1);
  sendATCommand("AT+QICLOSE=0", 500);
  sendATCommand("AT+QICLOSE=1", 500);
  sendATCommand("AT+QICLOSE=2", 500);
  delay(300);
  // eDRX keeps the RF front-end on a paging cycle and fights PSM for minimum sleep.
  sendATCommand("AT+CEDRXS=0,5", 1000);
  String psmStatus = sendATCommand("AT+CPSMS?", 1000);
  if (psmStatus.indexOf("1,") == -1 && psmStatus.indexOf(": 1") == -1) {
    String psmCmd = String("AT+CPSMS=1,,,\"") + PSM_PERIODIC_TAU + "\",\"" + PSM_ACTIVE_TIME + "\"";
    sendATCommand(psmCmd, 1000);
  }
  String sclkStatus = sendATCommand("AT+QSCLK?", 500);
  if (sclkStatus.indexOf("1") == -1) {
    sendATCommand("AT+QSCLK=1", 500);
  }
  modemFlushInput(50);
  oled_status("PSM ready", "", -1);
}

static void wakeModuleFromPSM(void) {
  if (!usePSM) return;
  Serial.println("Waking module from PSM...");
  oled_status("wake from", "PSM", -1);
  bc95_modem.print("AT\r\n");
  delay(100);
  modemFlushInput(50);
  String response = sendATCommand("AT", 1000);
  if (response.indexOf("OK") != -1) {
    Serial.println("Module woke from PSM");
    oled_status("PSM wake", "OK", -1);
  } else {
    delay(200);
    response = sendATCommand("AT", 1000);
    if (response.indexOf("OK") != -1) {
      oled_status("PSM wake", "OK", -1);
    } else {
      Serial.println("Warning: Module not responding after PSM wake");
      oled_status("PSM wake", "FAIL", -1);
    }
  }
  String ceregCheck = sendATCommand("AT+CEREG?", 1500);
  if (ceregCheck.indexOf(",1") != -1 || ceregCheck.indexOf(",5") != -1) {
    Serial.println("Network registration preserved after PSM");
  }
}

// Probe whether the modem answers "AT" at the currently-open UART baud.
static bool modemRespondsAT(void) {
  modemFlushInput(20);
  for (int i = 0; i < 3; i++) {
    String r = sendATCommand("AT", 400);
    if (r.indexOf("OK") != -1) return true;
    delay(80);
  }
  return false;
}

// Raise the modem UART from the fallback baud to BC95_BAUD. The AT+IPR reply
// comes back at the OLD baud, so we re-open the ESP UART at the target after.
static void raiseModemBaudToTarget(void) {
#if BC95_BAUD != BC95_BAUD_FALLBACK
  Serial.printf("[BAUD] raising modem UART %d -> %d\n", (int)BC95_BAUD_FALLBACK, (int)BC95_BAUD);
  sendATCommand(String("AT+IPR=") + String((unsigned long)BC95_BAUD), 800);
  sendATCommand("AT&W", 600);   // persist across power cycles (harmless if unsupported)
  delay(150);
  bc95_modem.begin(BC95_BAUD, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  delay(80);
  modemFlushInput(40);
  if (modemRespondsAT()) Serial.printf("[BAUD] modem now at %d\n", (int)BC95_BAUD);
  else Serial.printf("[BAUD] WARN: no OK at %d after AT+IPR\n", (int)BC95_BAUD);
#endif
}

// Cold-boot bring-up: open at the target baud and wait for the module to boot.
// If it never answers at the target, fall back to the fallback baud (fresh
// module / post-AT&F0) and migrate it to the target. Leaves the UART open at
// whichever baud the modem actually responded on.
static bool modemBootAndSetBaud(int maxAttempts, int delayMs) {
  bc95_modem.begin(BC95_BAUD, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  if (waitForModuleBoot(maxAttempts, delayMs)) return true;
#if BC95_BAUD != BC95_BAUD_FALLBACK
  Serial.printf("[BAUD] no reply at %d, trying fallback %d...\n",
                (int)BC95_BAUD, (int)BC95_BAUD_FALLBACK);
  bc95_modem.begin(BC95_BAUD_FALLBACK, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  if (waitForModuleBoot(maxAttempts, delayMs)) {
    raiseModemBaudToTarget();
    return true;
  }
#endif
  return false;
}

// Parse the <stat> field from a CEREG response. Handles both URC form
// ("+CEREG: <stat>[,<tac>,<ci>,<AcT>]") and solicited form
// ("+CEREG: <n>,<stat>[,...]"). Returns -1 if unparseable.
static int parseCeregStat(const String& resp) {
  int p = resp.lastIndexOf("+CEREG:");
  if (p < 0) return -1;
  int colon = resp.indexOf(':', p);
  if (colon < 0) return -1;
  int lineEnd = resp.indexOf('\n', colon);
  if (lineEnd < 0) lineEnd = resp.length();
  String rest = resp.substring(colon + 1, lineEnd);
  rest.trim();

  int c1 = rest.indexOf(',');
  String f0 = (c1 < 0) ? rest : rest.substring(0, c1);
  f0.trim();
  if (c1 < 0) return f0.toInt();

  String remainder = rest.substring(c1 + 1);
  int c2 = remainder.indexOf(',');
  String f1 = (c2 < 0) ? remainder : remainder.substring(0, c2);
  f1.trim();

  int v0 = f0.toInt();
  int v1 = f1.toInt();
  // Solicited ("<n>,<stat>") when f0 is a valid <n> (0..4) and f1 a valid <stat> (0..5).
  if (v0 >= 0 && v0 <= 4 && v1 >= 0 && v1 <= 5 && f1.length() > 0) return v1;
  return v0;
}

// Wait for EPS registration (CEREG stat 1 or 5), driven primarily by the
// +CEREG URC with a slow AT+CEREG? fallback. Replaces the previous loop that
// polled every 2 s up to 150 times (~300 s worst case) and kept MCU+modem
// fully awake the entire time.
static bool waitForCeregRegistered(unsigned long totalTimeoutMs) {
  // Subscribe to +CEREG URCs (n=2 = URC with TAC/CI).
  sendATCommand("AT+CEREG=2", 500);

  // Fast path: already registered.
  {
    String snap = sendATCommand("AT+CEREG?", 1500);
    int s = parseCeregStat(snap);
    if (s == 1 || s == 5) return true;
  }

  String buf;
  buf.reserve(256);
  unsigned long start = millis();
  unsigned long lastPoll = start;
  const unsigned long POLL_INTERVAL_MS = 15000;

  while (millis() - start < totalTimeoutMs) {
    // Drain URC bytes as they arrive.
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      buf += c;
      if (c == '\n') {
        int s = parseCeregStat(buf);
        if (s == 1 || s == 5) return true;
        if (buf.length() > 256) buf.remove(0, buf.length() - 128);
      }
    }
    // Fallback poll in case the URC is suppressed by the carrier.
    if (millis() - lastPoll > POLL_INTERVAL_MS) {
      lastPoll = millis();
      String poll = sendATCommand("AT+CEREG?", 1500);
      int s = parseCeregStat(poll);
      if (s == 1 || s == 5) return true;
    }
    delay(20);
  }
  return false;
}

static void initializeModule(void) {
  sendATCommand("AT", 500);
  sendATCommand("ATE0", 100);

  Serial.println("Checking network registration status...");
  String cgattCheck = sendATCommand("AT+CGATT?", 1500);
  String ceregCheck = sendATCommand("AT+CEREG?", 1500);

  bool alreadyAttached = (cgattCheck.indexOf("+CGATT: 1") != -1);
  bool alreadyRegistered = (ceregCheck.indexOf(",1") != -1 || ceregCheck.indexOf(",5") != -1);

  if (alreadyAttached && alreadyRegistered) {
    Serial.println("Module already registered! Minimal setup...");
    sendATCommand("AT+CMEE=2", 500);
    sendATCommand("AT+QSCLK=0", 500);
    sendATCommand("AT+CFUN=1", 2000);
    String cgattVerify = sendATCommand("AT+CGATT?", 1000);
    if (cgattVerify.indexOf("+CGATT: 1") == -1) {
      Serial.println("Warning: Network not attached, reattach...");
      sendATCommand("AT+CGATT=1", 5000);
      delay(2000);
    }
    modemFlushInput(20);
    moduleInitialized = true;
    Serial.println("Minimal setup complete. Module ready.");
    return;
  }

  Serial.println("Module not registered. Full initialization...");
  oled_status("init module", "start", -1);
  sendATCommand("AT+CMEE=2", 500);

#if BC95_FACTORY_RESET_ON_PROVISION
  // Opt-in clean slate. NOTE: AT&F0 wipes the persisted baud and APN, so the next cold boot
  // pays the 9600 fallback again and re-provisions. Only enable to recover a bad module.
  Serial.println("Factory reset (AT&F0)...");
  oled_status("module reset", "AT&F0", -1);
  sendATCommand("AT&F0", 1000);
#endif
  // Persist the target baud to the modem NV so future cold boots come straight up at
  // BC95_BAUD with no 9600 fallback. (raiseModemBaudToTarget also does this on first sync.)
#if BC95_BAUD != BC95_BAUD_FALLBACK
  sendATCommand(String("AT+IPR=") + String((unsigned long)BC95_BAUD), 800);
#endif
  Serial.println("Configuring APN (sensor.net)..."); //simky od Honzy sensor.net; moje simky iot.1nce.net
  sendATCommand("AT+QCGDEFCONT=\"IP\",\"sensor.net\"", 2000);
  sendATCommand("AT&W", 600);   // persist baud + APN + base config across power cycles/reboot
  Serial.println("Rebooting module (AT+NRB) to apply APN...");
  oled_status("module reboot", "wait", -1);
  sendATCommand("AT+NRB", 1000);
  Serial.println("Waiting for module reboot...");
  delay(4000);
#if BC95_BAUD != BC95_BAUD_FALLBACK
  // After NRB the module reboots at its persisted baud (BC95_BAUD if AT&W stuck). If a module
  // never persisted it (or a factory reset above wiped it), it comes back at the fallback —
  // re-sync and re-persist so the rest of init talks at the right rate.
  if (!modemRespondsAT()) {
    bc95_modem.begin(BC95_BAUD_FALLBACK, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
    delay(40);
    if (modemRespondsAT()) raiseModemBaudToTarget();
    else bc95_modem.begin(BC95_BAUD, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  }
#endif
  sendATCommand("AT", 500);
  sendATCommand("ATE0", 500);
  sendATCommand("AT+QSCLK=0", 500);
  Serial.println("Setting CFUN=1...");
  oled_status("set CFUN", "1", -1);
  sendATCommand("AT+CFUN=1", 3000);
  String cgsnResp = sendATCommand("AT+CGSN=1", 1000);
  int cgsnPos = cgsnResp.indexOf("+CGSN:");
  if (cgsnPos != -1) {
    int start = cgsnPos + 6;
    int end = cgsnResp.indexOf('\r', start);
    if (end == -1) end = cgsnResp.indexOf('\n', start);
    if (end == -1) end = cgsnResp.length();
    String imei = cgsnResp.substring(start, end);
    imei.trim();
    Serial.println("IMEI: " + imei);
  }
  Serial.println("Waiting for SIM (CPIN)...");
  oled_status("SIM wait", "CPIN", -1);
  bool simReady = false;
  for (int attempt = 0; attempt < 20 && !simReady; attempt++) {
    String cpin = sendATCommand("AT+CPIN?", 800);
    if (cpin.indexOf("READY") != -1) {
      simReady = true;
      break;
    }
    delay(500);
  }
  if (!simReady) {
    Serial.println("ERROR: SIM not ready");
    oled_status("SIM not", "ready", -1);
  }
  Serial.println("Attaching to network...");
  oled_status("attach", "CGATT=1", -1);
  sendATCommand("AT+CGATT=1", 1000);
  sendATCommand("AT+COPS=0", 630);

  // URC-driven registration wait: subscribes to +CEREG URCs and returns as soon
  // as the network reports stat=1 (home) or stat=5 (roaming). Worst-case awake
  // time is now bounded by the total timeout instead of 150 fixed polls.
  const unsigned long CEREG_TOTAL_TIMEOUT_MS = 120000UL;
  bool registered = waitForCeregRegistered(CEREG_TOTAL_TIMEOUT_MS);
  if (registered) {
    Serial.println("Network registered successfully!");
    String ceregSnap = sendATCommand("AT+CEREG?", 1500);
    if (parseCeregStat(ceregSnap) == 5) {
      oled_status("network roam", "OK", -1);
    } else {
      oled_status("network home", "OK", -1);
    }
    String cgpaddrVerify = sendATCommand("AT+CGPADDR", 1000);
  } else {
    Serial.println("ERROR: Failed to register on network");
    oled_status("network", "error timeout", -1);
  }
  String cgattResponse = sendATCommand("AT+CGATT?", 1000);
  moduleInitialized = registered;
  Serial.printf("Module init complete. Status: %s\n", moduleInitialized ? "OK" : "Failed");
}

// Phase 2b: optional sensors (MAX17048)
static bool initializeMAX17048(void) {
  Wire.beginTransmission(MAX17048_I2C_ADDR);
  if (Wire.endTransmission() != 0) return false;
  return true;
}
static float readMAX17048Voltage(void) {
  Wire.beginTransmission(MAX17048_I2C_ADDR);
  Wire.write(0x02);  // VCELL register
  if (Wire.endTransmission(true) != 0) return 0.0f;
  if (Wire.requestFrom((uint8_t)MAX17048_I2C_ADDR, (uint8_t)2) != 2) return 0.0f;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return (float)raw * 78.125f / 1000000.0f;  // 78.125uV per LSB
}
static float readMAX17048Percentage(void) {
  Wire.beginTransmission(MAX17048_I2C_ADDR);
  Wire.write(0x04);  // SOC register
  if (Wire.endTransmission(true) != 0) return 0.0f;
  if (Wire.requestFrom((uint8_t)MAX17048_I2C_ADDR, (uint8_t)2) != 2) return 0.0f;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return (float)(raw >> 8) + (float)(raw & 0xFF) / 256.0f;
}
static void readBatteryStatus(void) {
  static int lastValidPercentage = 0;
  if (!useFuelGauge) {
    batteryPercentage = 0;
    oledStatus.batteryPercent = 0;
    return;
  }
  float v = readMAX17048Voltage();
  float pct = readMAX17048Percentage();
  if (v <= 0.0f || pct <= 0.0f) {
    batteryPercentage = lastValidPercentage;
  } else {
    batteryPercentage = (int)constrain(pct, 0.0f, 100.0f);
    lastValidPercentage = batteryPercentage;
  }
  oledStatus.batteryPercent = batteryPercentage;
}
// Phase 2b: OLED transmit status (legacy name)
static void oled_layout_transmit(const char* status) {
  oled_status(status, "", -1);
}

// Phase 2b: NVS and device identity
static uint64_t generateDeviceId(void) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  const char* vendor_salt = "DFR0975_CAM_NBIOT_V0_SALT_2024";
  String input = "";
  for (int i = 0; i < 6; i++) {
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", mac[i]);
    input += hex;
  }
  input += vendor_salt;
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)input.c_str(), input.length());
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);
  uint64_t id = 0;
  for (int i = 0; i < 8; i++) {
    id = (id << 8) | hash[i];
  }
  return id;
}

static void generateMeasurementId(void) {
  uint8_t uuid_bytes[16];
  esp_fill_random(uuid_bytes, 16);
  uuid_bytes[6] = (uuid_bytes[6] & 0x0F) | 0x40;
  uuid_bytes[8] = (uuid_bytes[8] & 0x3F) | 0x80;
  snprintf(measurement_id, sizeof(measurement_id),
           "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
           uuid_bytes[0], uuid_bytes[1], uuid_bytes[2], uuid_bytes[3],
           uuid_bytes[4], uuid_bytes[5], uuid_bytes[6], uuid_bytes[7],
           uuid_bytes[8], uuid_bytes[9], uuid_bytes[10], uuid_bytes[11],
           uuid_bytes[12], uuid_bytes[13], uuid_bytes[14], uuid_bytes[15]);
}

static void initializeDeviceIdentity(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    Serial.printf("NVS init failed: %s\n", esp_err_to_name(err));
    return;
  }
  err = nvs_open("device_identity", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.printf("NVS open failed: %s\n", esp_err_to_name(err));
    return;
  }
  size_t required_size = sizeof(device_id);
  err = nvs_get_blob(nvs_handle, "device_id", &device_id, &required_size);
  if (err != ESP_OK) {
    device_id = generateDeviceId();
    Serial.printf("Generated new device ID: 0x%016llX\n", (unsigned long long)device_id);
    err = nvs_set_blob(nvs_handle, "device_id", &device_id, sizeof(device_id));
    if (err != ESP_OK) {
      Serial.printf("NVS set device_id failed: %s\n", esp_err_to_name(err));
    }
  } else {
    Serial.printf("Loaded device ID: 0x%016llX\n", (unsigned long long)device_id);
  }
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
}

#ifndef SLEEP_SEC_MIN
#define SLEEP_SEC_MIN 10
#endif
#ifndef SLEEP_SEC_MAX
#define SLEEP_SEC_MAX (86400 * 7)  // 7 days (within ESP deep-sleep range)
#endif

static int clampSleepSeconds(int sec) {
  if (sec < (int)SLEEP_SEC_MIN) return (int)SLEEP_SEC_MIN;
  if (sec > (int)SLEEP_SEC_MAX) return (int)SLEEP_SEC_MAX;
  return sec;
}

/**
 * Daytime deep-sleep duration when the PIR is the primary wake source: sleep
 * until the next NAP boundary so the day->night transition still happens even
 * with zero motion, while PIR wakes drive everything in between. Falls back to
 * a single periodic interval only if local network time is unknown this boot.
 */
static int daytimeFallbackSleepSeconds(void) {
  if (g_localSecOfDay < 0) {
    return clampSleepSeconds(DEFAULT_SLEEP_TIME);  // no network time -> periodic re-check
  }
  int elapsed = (int)((millis() - g_localTimeCapturedMs) / 1000UL);
  int nowSec = (g_localSecOfDay + elapsed) % 86400;
  if (nowSec < 0) nowSec += 86400;
  return clampSleepSeconds(secondsUntilNextNap(nowSec));
}

// If modem time says we are outside the daytime window, take one long deep sleep
// until the next WAKE boundary. No camera warm-up, GNSS, or ML on this wake —
// avoids repeated timer wakes at DEFAULT_SLEEP_TIME that only re-check the clock.
/** Ask the modem for CCLK and convert to local second-of-day. Returns -1 when
 *  the modem has no usable time (not registered yet / invalid CCLK). */
static int gateTimeFromModem(void) {
  int gateHour = -1, gateMin = -1;
  String gateRaw;
  if (!queryNetworkTime(gateHour, gateMin, gateRaw)) return -1;
  int q1 = gateRaw.indexOf('"');
  int q2 = (q1 >= 0) ? gateRaw.indexOf('"', q1 + 1) : -1;
  String payload = (q1 >= 0 && q2 > q1) ? gateRaw.substring(q1, q2 + 1) : "";
  NetworkTime nt;
  if (!parseNetworkTimeFull(payload, nt)) {
    Serial.printf("[SCHED] CCLK time invalid (%s).\n", gateRaw.c_str());
    return -1;
  }
  int localSec = resolveLocalSecondOfDay(nt);
  Serial.printf("[SCHED] Local time: %02d:%02d (TZ: %s%+d qh)\n",
                localSec / 3600, (localSec % 3600) / 60,
                nt.hasTz ? "modem " : "fallback ",
                nt.hasTz ? nt.tzQuarterHours : fallbackTzQuarterHours(nt));
  return localSec;
}

static void tryScheduleNightSleepIfOutsideWindow(void) {
  // Resolve the local hour from the cheapest trustworthy source. Previously this
  // gave up when the modem had no time yet (always the case on a cold boot,
  // because registration happens later) and ran a FULL camera+ML+upload job at
  // any hour — silently defeating the configured wake/nap window.
  int localSec = gateTimeFromModem();
  const char* src = "modem";

  if (localSec < 0 && rtcLocalSecondOfDay(localSec)) {
    // Clock seeded on an earlier wake and kept by the RTC through deep sleep.
    src = "rtc";
    // (no radio used on this path)
    Serial.printf("[SCHED] Local time %02d:%02d (persisted RTC clock, no radio)\n",
                  localSec / 3600, (localSec % 3600) / 60);
  }

  if (localSec < 0) {
    // Cold boot with no clock anywhere. Bring the modem up and ask again: every
    // non-night wake path needs the modem anyway (upload or settings poll), so
    // this costs nothing during the day — the later initializeModule() call is
    // then a no-op — and at night it saves a whole camera+GNSS+ML cycle.
    Serial.println("[SCHED] No clock yet — registering modem to resolve the hour…");
    initializeModule();
    localSec = gateTimeFromModem();
    src = "modem(cold)";
  }

  if (localSec < 0) {
    Serial.println("[SCHED] Time still unknown — full init (camera + ML).");
    return;
  }

  persistLocalClock(localSec);           // survives deep sleep for later wakes
  g_localSecOfDay = localSec;            // for daytime "sleep until NAP" fallback
  g_localTimeCapturedMs = millis();
  int lh = localSec / 3600;
  int lm = (localSec % 3600) / 60;
  oledStatus.networkHour = lh;          // seed OLED clock early so it's never blank
  oledStatus.networkMinute = lm;

  if (isInsideOperationalWindow(localSec)) {
    Serial.printf("[SCHED] Inside window (%02u:%02u-%02u:%02u) [%s]. Proceeding.\n",
                  g_sleepGeo.wake_hour, g_sleepGeo.wake_minute,
                  g_sleepGeo.nap_hour, g_sleepGeo.nap_minute, src);
    return;
  }

  int sleepSec = secondsUntilNextWake(localSec);
  sleepSec = clampSleepSeconds(sleepSec);
  Serial.printf("[SCHED] Night [%s] — single %ds deep sleep until next WAKE (no camera/GNSS/ML this wake).\n",
                src, sleepSec);
  s_sleepOledExtra[0] = '\0';
  customSleepDuration = sleepSec;
  enterNightScheduleSleepMode();
}

static void sleepGeoApplyDefaults(SleepGeoConfig& c) {
  c.magic = kSleepGeoMagic;
  c.home_lat = 49.012192;
  c.home_lon = 14.539031;
  c.home_radius_m = 1000;
  c.sleep_home_s = 1800;
  c.sleep_away_s = 600;
  // Operational window + PIR modes default to the previous compile-time
  // behaviour (wake 07:00, nap 20:00, PIR armed in both daytime states).
  c.wake_hour = WAKE_HOUR;
  c.wake_minute = WAKE_MINUTE;
  c.nap_hour = NAP_HOUR;
  c.nap_minute = NAP_MINUTE;
  c.pir_home_mode = PIR_MODE_PIR_TIMER;
  c.pir_away_mode = PIR_MODE_PIR_TIMER;
}

static void loadSleepGeoFromNvs(void) {
  sleepGeoApplyDefaults(g_sleepGeo);
  nvs_handle_t h;
  esp_err_t err = nvs_open("sleep_geo", NVS_READONLY, &h);
  if (err != ESP_OK) {
    Serial.printf("[SLEEP_GEO] NVS open (ro) %s — defaults\n", esp_err_to_name(err));
    return;
  }
  SleepGeoConfig tmp{};
  size_t sz = sizeof(tmp);
  err = nvs_get_blob(h, "cfg", &tmp, &sz);
  nvs_close(h);
  if (err != ESP_OK || sz != sizeof(SleepGeoConfig) || tmp.magic != kSleepGeoMagic) {
    Serial.println("[SLEEP_GEO] No valid cfg — defaults");
    return;
  }
  g_sleepGeo = tmp;
  Serial.printf("[SLEEP_GEO] Loaded: r=%lum home=%lus away=%lus lat=%.6f lon=%.6f win=%02u:%02u-%02u:%02u pir(home=%u,away=%u)\n",
                (unsigned long)g_sleepGeo.home_radius_m,
                (unsigned long)g_sleepGeo.sleep_home_s, (unsigned long)g_sleepGeo.sleep_away_s,
                g_sleepGeo.home_lat, g_sleepGeo.home_lon,
                g_sleepGeo.wake_hour, g_sleepGeo.wake_minute, g_sleepGeo.nap_hour, g_sleepGeo.nap_minute,
                g_sleepGeo.pir_home_mode, g_sleepGeo.pir_away_mode);
}

static bool saveSleepGeoToNvs(const SleepGeoConfig& c) {
  nvs_handle_t h;
  esp_err_t err = nvs_open("sleep_geo", NVS_READWRITE, &h);
  if (err != ESP_OK) {
    Serial.printf("[SLEEP_GEO] NVS open (rw) failed: %s\n", esp_err_to_name(err));
    return false;
  }
  SleepGeoConfig toWrite = c;
  toWrite.magic = kSleepGeoMagic;
  err = nvs_set_blob(h, "cfg", &toWrite, sizeof(toWrite));
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  if (err != ESP_OK) {
    Serial.printf("[SLEEP_GEO] NVS save failed: %s\n", esp_err_to_name(err));
    return false;
  }
  g_sleepGeo = toWrite;
  return true;
}

/** Great-circle distance in meters (WGS84 approximate). */
static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusM = 6371000.0;
  const double deg2rad = std::acos(-1.0) / 180.0;
  double p1 = lat1 * deg2rad;
  double p2 = lat2 * deg2rad;
  double dLat = (lat2 - lat1) * deg2rad;
  double dLon = (lon2 - lon1) * deg2rad;
  double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
             std::cos(p1) * std::cos(p2) * std::sin(dLon / 2) * std::sin(dLon / 2);
  double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

/** Configure customSleepDuration + s_daytimePirArm for a geofence state (home or
 *  away) from that state's pir mode and sleep interval:
 *    PIR+timer  -> wake on motion OR after sleep_X_s (clamped to the NAP boundary)
 *    timer-only -> wake only after sleep_X_s (clamped to NAP), PIR disarmed
 *    PIR-only   -> wake only on motion; timer = PIR_ONLY_SAFETY_S (clamped to NAP)
 *  The NAP clamp guarantees the device never sleeps past the day->night boundary. */
static void configureDaytimeSleepForState(bool isHome) {
  const uint8_t mode = isHome ? g_sleepGeo.pir_home_mode : g_sleepGeo.pir_away_mode;
  const uint32_t interval = isHome ? g_sleepGeo.sleep_home_s : g_sleepGeo.sleep_away_s;
  const int untilNap = daytimeFallbackSleepSeconds();
  const char* tag;
  if (mode == PIR_MODE_PIR_ONLY) {
    int t = clampSleepSeconds((int)PIR_ONLY_SAFETY_S);
    customSleepDuration = (t < untilNap) ? t : untilNap;
    s_daytimePirArm = true;
    tag = isHome ? "home PIRonly" : "away PIRonly";
  } else {
    int t = clampSleepSeconds((int)interval);
    customSleepDuration = (t < untilNap) ? t : untilNap;
    s_daytimePirArm = (mode == PIR_MODE_PIR_TIMER);
    tag = (mode == PIR_MODE_PIR_TIMER) ? (isHome ? "home PIR+T" : "away PIR+T")
                                       : (isHome ? "home timer" : "away timer");
  }
  snprintf(s_sleepOledExtra, sizeof(s_sleepOledExtra), "%s", tag);
  Serial.printf("[SLEEP_GEO] %s -> mode=%u timer=%ds pir=%d\n",
                isHome ? "home" : "away", mode, customSleepDuration, (int)s_daytimePirArm);
}

/** Set customSleepDuration for daytime deep sleep, honouring the per-state PIR
 *  mode + interval. Not for the night window. */
static void applyDaytimeSleepFromGeofence(void) {
  s_sleepOledExtra[0] = '\0';
  const bool fixOk = (HAS_GNSS != 0) && g_gnssLastFix.valid;

  if (g_sleepGeo.home_radius_m == 0) {
    // Geofence disabled: no home/away distinction — treat as deployed (away).
    configureDaytimeSleepForState(false);
    return;
  }

  if (!fixOk) {
    // No GNSS fix: short retry, keep PIR armed so motion can still wake.
    customSleepDuration = clampSleepSeconds((int)GNSS_NO_FIX_RETRY_S);
    s_daytimePirArm = true;
    Serial.printf("[SLEEP_GEO] no_fix -> retry %ds\n", customSleepDuration);
    snprintf(s_sleepOledExtra, sizeof(s_sleepOledExtra), "no GPS");
    return;
  }

  double dist = haversineMeters(g_gnssLastFix.lat_deg, g_gnssLastFix.lon_deg,
                                g_sleepGeo.home_lat, g_sleepGeo.home_lon);
  configureDaytimeSleepForState(dist <= (double)g_sleepGeo.home_radius_m);
}

/** GNSS geofence gate before camera/ML (daytime wakes only; night handled earlier).
 *  In depot → settings-only TCP poll, then deep sleep with sleep_home_s (no capture).
 *  No fix → deep sleep GNSS_NO_FIX_RETRY_S (10 min).
 *  Outside home radius → return and run full observation loop. */
static void tryEarlyGeofenceSleepRouting(void) {
  if (g_sleepGeo.home_radius_m == 0) {
    Serial.println("[SLEEP_GEO] geofence disabled — full observation loop");
    return;
  }
#if !HAS_GNSS
  Serial.println("[SLEEP_GEO] GNSS disabled — full observation loop");
  return;
#else
  pinMode(SIOD_GPIO_NUM, INPUT_PULLUP);
  pinMode(SIOC_GPIO_NUM, INPUT_PULLUP);
  delay(5);
  i2cBusRecover();
  initI2CBusOnce();

  // Bring the OLED up now (same GPIO1/2 bus) so the operator sees the device is
  // alive during the long GNSS acquire below, instead of a dark panel for ~60s.
  if (oled_bring_up_early()) {
    oled_boot_loading(1200);            // brief boot animation: "device is up"
    Wire.setClock(I2C_CLOCK_HZ);        // u8g2.begin() may have changed bus clock
  }

  Serial.println("[SLEEP_GEO] Early GNSS check (before camera)...");
  GnssFix fix;
  // Full-screen "searching for GNSS" animation runs during the blocking read.
  bool gotFix = gnssReadLatestFixSinceWake(fix, GNSS_ACQUIRE_TIMEOUT_MS, gnssSearchAnimTick);
  g_gnssLastFix = fix;

  Wire.end();
  i2c_bus_initialized = false;
  pinMode(SIOD_GPIO_NUM, INPUT);
  pinMode(SIOC_GPIO_NUM, INPUT);
  delay(10);

  if (!gotFix || !g_gnssLastFix.valid) {
    customSleepDuration = clampSleepSeconds((int)GNSS_NO_FIX_RETRY_S);
    s_daytimePirArm = true;   // motion can still wake during the no-fix retry
    Serial.printf("[SLEEP_GEO] early no_fix -> sleep %ds (retry)\n", customSleepDuration);
    snprintf(s_sleepOledExtra, sizeof(s_sleepOledExtra), "no GPS");
    enterDaytimeSleepMode();
  }

  double dist = haversineMeters(g_gnssLastFix.lat_deg, g_gnssLastFix.lon_deg,
                                g_sleepGeo.home_lat, g_sleepGeo.home_lon);
  if (dist <= (double)g_sleepGeo.home_radius_m) {
    Serial.printf("[SLEEP_GEO] early home (%.1fm) skip camera/ML — settings poll\n", dist);
#if BENCH_SKIP_UPLINK
    Serial.println("[BENCH] SKIP_UPLINK: skipping depot settings poll");
#else
    // OLED was torn down with the GNSS I2C teardown above; restore for status UI.
    if (s_oled_initialized) {
      initI2CBusOnce();
    }
    (void)pollSettingsOverTCP();
#if OTA_VIA_SERVER_COMMAND
    if (g_otaRequested) {
      g_otaRequested = false;
      runOtaUpdate();
    }
#endif
#if OTA_CHECK_ON_COLD_BOOT
    // Cold-boot OTA is normally deferred until after the main job; depot never
    // reaches that path, so run the check here while the modem is already online.
    if (g_otaColdBootPending) {
      g_otaColdBootPending = false;
      Serial.println("[OTA] depot-path cold-boot update check");
      runOtaUpdate();
    }
#endif
    // Web-app settings may have moved home / disabled the geofence — re-check.
    if (g_sleepGeo.home_radius_m == 0) {
      Serial.println("[SLEEP_GEO] geofence disabled by settings — full observation loop");
      return;
    }
    dist = haversineMeters(g_gnssLastFix.lat_deg, g_gnssLastFix.lon_deg,
                           g_sleepGeo.home_lat, g_sleepGeo.home_lon);
    if (dist > (double)g_sleepGeo.home_radius_m) {
      Serial.printf("[SLEEP_GEO] outside after settings (%.1fm) — full observation loop\n", dist);
      return;
    }
#endif
    configureDaytimeSleepForState(true);   // in-depot: use (possibly updated) home interval
    enterDaytimeSleepMode();
  }

  Serial.printf("[SLEEP_GEO] early away (%.1fm) -> full observation loop\n", dist);
#endif
}

static String asciiToHex(const String& ascii) {
  String hex;
  hex.reserve(ascii.length() * 2);
  for (size_t i = 0; i < ascii.length(); i++) {
    char buffer[3];
    snprintf(buffer, sizeof(buffer), "%02X", (uint8_t)ascii[i]);
    hex += buffer;
  }
  return hex;
}

static String buildLineProtocolASCII(void) {
  String lineProtocol;
  lineProtocol.reserve(200);
  lineProtocol = "gps_data,device=esp32 ";
  lineProtocol += "temp=" + String(sensorDataValid ? sensorData.temperature : NAN, 1) + ",";
  lineProtocol += "hum=" + String(sensorDataValid ? sensorData.humidity : NAN, 1) + ",";
  lineProtocol += "bat=" + String(batteryPercentage) + ",";
  // GNSS fields
  lineProtocol += "gnss_valid=" + String(g_gnssLastFix.valid ? 1 : 0) + ",";
  if (g_gnssLastFix.valid) {
    lineProtocol += "gnss_lat=" + String(g_gnssLastFix.lat_deg, 7) + ",";
    lineProtocol += "gnss_lon=" + String(g_gnssLastFix.lon_deg, 7) + ",";
    lineProtocol += "gnss_alt_m=" + String(g_gnssLastFix.altitude_m, 2) + ",";
    lineProtocol += "gnss_sats=" + String(g_gnssLastFix.sats_used) + ",";
    lineProtocol += "gnss_hdop=" + String(g_gnssLastFix.hdop, 2) + ",";
    lineProtocol += "gnss_spd_kn=" + String(g_gnssLastFix.speed_knots, 2) + ",";
    lineProtocol += "gnss_course=" + String(g_gnssLastFix.course_deg, 1) + ",";
  } else {
    lineProtocol += "gnss_lat=999,";
    lineProtocol += "gnss_lon=999,";
    lineProtocol += "gnss_alt_m=999,";
    lineProtocol += "gnss_sats=0,";
    lineProtocol += "gnss_hdop=999,";
    lineProtocol += "gnss_spd_kn=0,";
    lineProtocol += "gnss_course=0,";
  }
  lineProtocol += "fill=" + String(fillLevel) + ",";
  lineProtocol += "mat=\"" + detectedMaterial + "\",";
  lineProtocol += "lvl_conf=" + String(levelConfidence, 3) + ",";
  lineProtocol += "mat_conf=" + String(materialConfidence, 3) + ",";
  lineProtocol += "all_conf=" + String(overallConfidence, 3) + ",";
  for (int i = 0; i < 13; i++) {
    if (materialClassConfidences[i] > 0.001f) {
      const char* names[] = {"aac_conf", "asphalt_conf", "bricks_conf", "concrete_conf", "empty_conf",
                             "glass_conf", "gypsum_conf", "misc_conf", "polystyrene_conf", "soil_conf",
                             "steel_conf", "wood_conf", "wool_conf"};
      lineProtocol += String(names[i]) + "=" + String(materialClassConfidences[i], 3) + ",";
    }
  }
  if (lineProtocol.endsWith(",")) lineProtocol.remove(lineProtocol.length() - 1);
  return lineProtocol;
}

// TCP receive: poll for incoming data using AT+QIRD
static bool receiveFromTCP(int connectID, String& outData, unsigned long timeoutMs) {
  unsigned long start = millis();
  outData = "";
  while (millis() - start < timeoutMs) {
    String readCmd = "AT+QIRD=" + String(connectID) + "," + String(TCP_RECV_BUFFER_SIZE);
    String response = sendATCommand(readCmd, 2000);
    int qirdPos = response.indexOf("+QIRD:");
    if (qirdPos != -1) {
      int colonPos = response.indexOf(':', qirdPos);
      int crPos = response.indexOf('\r', colonPos);
      if (crPos == -1) crPos = response.indexOf('\n', colonPos);
      String lenStr = (crPos > colonPos) ? response.substring(colonPos + 1, crPos) : response.substring(colonPos + 1);
      lenStr.trim();
      int dataLen = lenStr.toInt();
      if (dataLen > 0) {
        int dataStart = response.indexOf('\n', qirdPos);
        if (dataStart != -1) {
          outData = response.substring(dataStart + 1);
          outData.trim();
          int okPos = outData.lastIndexOf("OK");
          if (okPos > 0) outData = outData.substring(0, okPos);
          outData.trim();
        }
        return true;
      }
    }
    delay(TCP_RECV_POLL_MS);
  }
  return false;
}

// Set by the set_config command branch when the config was applied AND saved
// to NVS. sendPhotoOverTCP / pollSettingsOverTCP write a "CFG_APPLIED\n" ack
// back on the socket so the server only clears the pending config on real,
// confirmed delivery (a lost reply keeps it pending → automatically re-sent
// next cycle).
static bool g_cfgAppliedAck = false;

// Parse JSON command from server and apply it
static void parseAndApplyCommand(const String& jsonData) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, jsonData);
  if (err) {
    Serial.printf("JSON parse error: %s\n", err.c_str());
    oled_status("CMD Parse", "Error", -1);
    return;
  }
  const char* cmd = doc["cmd"] | "";
  if (strcmp(cmd, "set_sleep") == 0) {
    int v = doc["value"] | DEFAULT_SLEEP_TIME;
    int newSleep = clampSleepSeconds(v);
    g_sleepGeo.sleep_home_s = (uint32_t)newSleep;
    g_sleepGeo.sleep_away_s = (uint32_t)newSleep;
    if (saveSleepGeoToNvs(g_sleepGeo)) {
      Serial.printf("Command: set_sleep=%d (NVS home+away)\n", newSleep);
    } else {
      Serial.printf("Command: set_sleep=%d (NVS save failed)\n", newSleep);
    }
    char msg[24];
    snprintf(msg, sizeof(msg), "sleep=%d s", newSleep);
    oled_status("CMD Received", msg, -1);
    delay(2000);
  } else if (strcmp(cmd, "set_sleep_geo") == 0) {
    SleepGeoConfig next = g_sleepGeo;
    if (!doc["home_lat"].isNull()) next.home_lat = doc["home_lat"].as<double>();
    if (!doc["home_lon"].isNull()) next.home_lon = doc["home_lon"].as<double>();
    if (!doc["home_radius_m"].isNull()) {
      uint32_t r = (uint32_t)doc["home_radius_m"].as<unsigned long>();
      if (r > 2000000UL) r = 2000000UL;
      next.home_radius_m = r;
    }
    if (!doc["sleep_home"].isNull()) {
      int s = clampSleepSeconds((int)doc["sleep_home"].as<long>());
      next.sleep_home_s = (uint32_t)s;
    }
    if (!doc["sleep_away"].isNull()) {
      int s = clampSleepSeconds((int)doc["sleep_away"].as<long>());
      next.sleep_away_s = (uint32_t)s;
    }
    if (saveSleepGeoToNvs(next)) {
      Serial.printf("Command: set_sleep_geo r=%lu home=%lus away=%lus\n",
                    (unsigned long)g_sleepGeo.home_radius_m,
                    (unsigned long)g_sleepGeo.sleep_home_s,
                    (unsigned long)g_sleepGeo.sleep_away_s);
      oled_status("CMD", "sleep_geo OK", -1);
    } else {
      oled_status("CMD", "sleep_geo NVS", -1);
    }
    delay(2000);
  } else if (strcmp(cmd, "set_config") == 0) {
    // Unified config push from /app/ device settings. Any subset of fields may
    // be present; missing fields keep their current value. Ranges clamped here
    // as a second line of defence (server validates too).
    SleepGeoConfig next = g_sleepGeo;
    auto u8clamp = [](long v, long hi) -> uint8_t {
      if (v < 0) v = 0; if (v > hi) v = hi; return (uint8_t)v;
    };
    if (!doc["wake_hour"].isNull())    next.wake_hour   = u8clamp(doc["wake_hour"].as<long>(), 23);
    if (!doc["wake_minute"].isNull())  next.wake_minute = u8clamp(doc["wake_minute"].as<long>(), 59);
    if (!doc["nap_hour"].isNull())     next.nap_hour    = u8clamp(doc["nap_hour"].as<long>(), 23);
    if (!doc["nap_minute"].isNull())   next.nap_minute  = u8clamp(doc["nap_minute"].as<long>(), 59);
    if (!doc["home_lat"].isNull())     next.home_lat    = doc["home_lat"].as<double>();
    if (!doc["home_lon"].isNull())     next.home_lon    = doc["home_lon"].as<double>();
    if (!doc["home_radius_m"].isNull()) {
      uint32_t r = (uint32_t)doc["home_radius_m"].as<unsigned long>();
      if (r > 2000000UL) r = 2000000UL;
      next.home_radius_m = r;
    }
    if (!doc["sleep_home_s"].isNull()) next.sleep_home_s = (uint32_t)clampSleepSeconds((int)doc["sleep_home_s"].as<long>());
    if (!doc["sleep_away_s"].isNull()) next.sleep_away_s = (uint32_t)clampSleepSeconds((int)doc["sleep_away_s"].as<long>());
    if (!doc["pir_home_mode"].isNull()) next.pir_home_mode = u8clamp(doc["pir_home_mode"].as<long>(), 2);
    if (!doc["pir_away_mode"].isNull()) next.pir_away_mode = u8clamp(doc["pir_away_mode"].as<long>(), 2);

    if (saveSleepGeoToNvs(next)) {
      g_sleepGeo = next;  // apply in RAM immediately (also persisted for next boot)
      g_cfgAppliedAck = true;   // confirm delivery to the server on this socket
      Serial.printf("Command: set_config applied win=%02u:%02u-%02u:%02u r=%lu home=%lus away=%lus pir(%u,%u) lat=%.6f lon=%.6f\n",
                    g_sleepGeo.wake_hour, g_sleepGeo.wake_minute, g_sleepGeo.nap_hour, g_sleepGeo.nap_minute,
                    (unsigned long)g_sleepGeo.home_radius_m,
                    (unsigned long)g_sleepGeo.sleep_home_s, (unsigned long)g_sleepGeo.sleep_away_s,
                    g_sleepGeo.pir_home_mode, g_sleepGeo.pir_away_mode,
                    g_sleepGeo.home_lat, g_sleepGeo.home_lon);
      oled_status("CMD", "config OK", -1);
    } else {
      Serial.println("Command: set_config NVS save failed");
      oled_status("CMD", "config NVS", -1);
    }
    delay(2000);
  } else if (strcmp(cmd, "ack") == 0) {
    Serial.println("Command: ack (no change)");
    oled_status("CMD Received", "ACK", -1);
    delay(1000);
  } else if (strcmp(cmd, "ota") == 0) {
    // Defer the actual update until the telemetry socket is closed (runMainJob).
    g_otaRequested = true;
    Serial.println("Command: ota (deferred to end of cycle)");
    oled_status("CMD Received", "OTA queued", -1);
    delay(1000);
  } else {
    Serial.printf("Unknown command: %s\n", cmd);
    oled_status("CMD Unknown", cmd, -1);
    delay(1000);
  }
}

/** Wait for a server JSON command on an open TCP socket and apply it.
 *  On set_config success, sends CFG_APPLIED so the server clears pending state. */
static void applyTcpServerReply(int connectID) {
#if TCP_WAIT_FOR_SERVER_RESPONSE
  oled_status("Waiting for", "server...", -1);
  String serverResponse;
  if (receiveFromTCP(connectID, serverResponse, TCP_RECV_TIMEOUT_MS)) {
    Serial.println("Server response: " + serverResponse);
    parseAndApplyCommand(serverResponse);
    if (g_cfgAppliedAck) {
      g_cfgAppliedAck = false;
      static const char kAck[] = "CFG_APPLIED\n";
      if (sendBinaryOverTCP(connectID, (const uint8_t*)kAck, sizeof(kAck) - 1, 5000, 10000)) {
        Serial.println("CFG_APPLIED ack sent");
      } else {
        Serial.println("CFG_APPLIED ack send failed (config stays pending server-side)");
      }
      delay(300);
    }
  } else {
    Serial.println("No response from server (timeout)");
    oled_status("No server", "response", -1);
    delay(1000);
  }
#else
  (void)connectID;
  Serial.println("TCP: skip wait for server response (TCP_WAIT_FOR_SERVER_RESPONSE=0)");
#endif
}

/** Depot-path settings poll: open telemetry TCP, announce device with image_size:0
 *  (no photo / no line-protocol telemetry), then apply any pending web-app config.
 *  Server must accept a zero-byte image upload and still push set_config. */
static bool pollSettingsOverTCP(void) {
  Serial.println("[CFG] settings-only poll starting...");
  oled_status("Settings", "check...", 0);
  generateMeasurementId();

  if (!moduleInitialized) {
    oled_status("Settings", "modem...", 0);
    initializeModule();
  }
  if (!moduleInitialized) {
    Serial.println("[CFG] modem not ready — skip settings poll");
    oled_status("Settings", "modem fail", -1);
    delay(1000);
    return false;
  }

  int contextID = 1;
  int connectID = 0;
  // Buffer access mode (0): same as photo path — replies are read via AT+QIRD.
  int accessMode = 0;
  String openCmd = String("AT+QIOPEN=") + String(contextID) + "," + String(connectID) +
                   ",\"TCP\",\"" + SERVER_IP + "\"," + String(TCP_PORT) + ",0," + String(accessMode);
  String response = sendATCommand(openCmd, 10000);
  if (response.indexOf("OK") == -1) {
    Serial.println("[CFG] QIOPEN issue failed");
    oled_status("Settings", "TCP OPEN ERR", -1);
    return false;
  }
  if (!waitForQIOPEN(connectID, 60000, response)) {
    Serial.println("[CFG] QIOPEN timeout");
    oled_status("Settings", "TCP TIMEOUT", -1);
    return false;
  }
  Serial.println("[CFG] TCP connected — sending settings-only header");
  oled_status("Settings", "polling...", 20);

  String header = "CORRELATION_HEADER\n";
  header += "app:container-classifier\n";
  header += "device_id:" + String((unsigned long long)device_id, HEX) + "\n";
  header += "measurement_id:" + String(measurement_id) + "\n";
  header += "timestamp:" + String(millis()) + "\n";
  header += "image_size:0\n";
  header += "settings_only:1\n";
  header += "END_HEADER\n";
  if (!sendBinaryOverTCP(connectID, (const uint8_t*)header.c_str(), header.length(), 5000, 10000)) {
    Serial.println("[CFG] header SEND FAIL");
    oled_status("Settings", "SEND FAIL", -1);
    sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
    return false;
  }

  applyTcpServerReply(connectID);

  sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
  oled_status("Settings", "done", 100);
  Serial.println("[CFG] settings-only poll complete");
  return true;
}

// ============================================================================
// OTA over NB-IoT (raw TCP, length-framed)
// ----------------------------------------------------------------------------
// Reuses the Quectel "QI" TCP/IP stack already used for telemetry. The device
// opens a socket to the OTA server, sends one request line, and the server
// replies with either:
//     OTA_NONE\n                                  (already up to date)
//   or
//     OTA_BEGIN size=<bytes> ver=<n> md5=<hex>\n   followed by exactly <bytes>
//                                                  of the raw firmware .bin
// The body is streamed straight into the inactive OTA slot via esp_ota_*,
// MD5-verified end-to-end, then the device reboots into the new image. The
// ESP32 bootloader validates the image (magic/checksum) on esp_ota_end(), so a
// truncated or corrupt download can never brick the device — it just keeps
// running the current slot.
// ============================================================================

// Small framed reader over AT+QIRD (socket opened in buffer access mode).
struct OtaSock {
  int id;
  uint8_t rx[1200];
  int rxLen;
  int rxPos;
};

static void otaCloseSocket(int connectID) {
  sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
}

// Read up to maxLen bytes from a TCP socket via AT+QIRD. Binary-safe: parses the
// ASCII "+QIRD: <len>\r\n" header, then reads exactly <len> raw bytes straight
// from the UART (NOT through Arduino String, which truncates on embedded 0x00).
// Returns byte count, 0 if nothing buffered yet, -1 on modem error.
static int qirdReadBytes(int connectID, uint8_t* buf, size_t maxLen, unsigned long timeoutMs) {
  modemFlushInput(5);
  String readCmd = String("AT+QIRD=") + String(connectID) + "," + String((unsigned)maxLen);
  bc95_modem.print(readCmd + "\r\n");

  // Phase 1: read the ASCII header line "+QIRD: <len>" (ends at its '\n').
  String hdr;
  hdr.reserve(48);
  long n = -1;
  bool haveLen = false;
  unsigned long start = millis();
  while (!haveLen && (millis() - start) < timeoutMs) {
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      hdr += c;
      if (hdr.indexOf("+CME ERROR") != -1) return -1;
      if (hdr.indexOf("\r\nERROR") != -1 && hdr.indexOf("+QIRD:") == -1) return -1;
      int p = hdr.indexOf("+QIRD:");
      if (p != -1) {
        int nl = hdr.indexOf('\n', p);
        if (nl != -1) {
          String ls = hdr.substring(p + 6, nl);
          ls.trim();
          n = ls.toInt();
          haveLen = true;
          break;
        }
      }
    }
    if (!haveLen) delay(2);
  }
  if (!haveLen) return 0;          // modem slow / no URC yet — caller retries
  if (n <= 0) {
    modemFlushInput(20);           // "+QIRD: 0" -> swallow trailing OK
    return 0;
  }
  if (n > (long)maxLen) n = (long)maxLen;

  // Phase 2: read exactly n raw payload bytes (may contain CR/LF/NUL).
  int got = 0;
  unsigned long bstart = millis();
  while (got < n && (millis() - bstart) < (timeoutMs + 8000)) {
    while (bc95_modem.available() && got < n) {
      buf[got++] = (uint8_t)bc95_modem.read();
    }
    if (got < n) delay(2);
  }
  modemFlushInput(30);             // swallow trailing \r\nOK\r\n
  return got;
}

// Block until exactly n bytes are read (across multiple QIRD batches) or the
// overall budget expires. NB-IoT delivers in slow bursts, so an empty QIRD
// (+QIRD: 0) or a transient ERROR is normal — keep polling until the budget,
// don't abort the whole multi-minute download on a single hiccup.
static bool otaReadExact(OtaSock& s, uint8_t* dst, int n, unsigned long overallMs) {
  unsigned long start = millis();
  int got = 0;
  while (got < n) {
    if (s.rxPos >= s.rxLen) {
      int r = qirdReadBytes(s.id, s.rx, OTA_QIRD_READ_LEN, 8000);
      if (r > 0) {
        s.rxLen = r;
        s.rxPos = 0;
      } else {
        s.rxLen = 0;
        s.rxPos = 0;
        if (millis() - start > overallMs) return false;  // nothing for too long
        delay(OTA_RX_POLL_MS);
        continue;
      }
    }
    int avail = s.rxLen - s.rxPos;
    int take = (n - got < avail) ? (n - got) : avail;
    memcpy(dst + got, s.rx + s.rxPos, take);
    s.rxPos += take;
    got += take;
  }
  return true;
}

// Read the single ASCII header line (OTA_BEGIN / OTA_NONE) terminated by '\n',
// using the same binary-safe QIRD reader as the firmware download. Crucially,
// any bytes AFTER the '\n' (the first firmware bytes, which can arrive in the
// same QIRD batch as the header) are left in s.rx at s.rxPos so otaReadExact()
// consumes them next — no byte is lost or duplicated at the ASCII↔binary seam.
// Using a String here for the binary tail (the old approach) corrupted the seam:
// it truncated on 0x00 and leaked the framing '\r' as the first written byte
// ("invalid magic byte ... saw 0x0d").
static bool otaReadHeaderLine(int connectID, OtaSock& s, String& line, unsigned long overallMs) {
  line = "";
  s.rxLen = 0;
  s.rxPos = 0;
  unsigned long start = millis();
  for (;;) {
    if (s.rxPos >= s.rxLen) {
      int r = qirdReadBytes(connectID, s.rx, OTA_QIRD_READ_LEN, 8000);
      if (r > 0) {
        s.rxLen = r;
        s.rxPos = 0;
      } else {
        s.rxLen = 0;
        s.rxPos = 0;
        if (millis() - start > overallMs) return false;
        delay(OTA_RX_POLL_MS);
        continue;
      }
    }
    while (s.rxPos < s.rxLen) {
      char c = (char)s.rx[s.rxPos++];
      if (c == '\n') {            // s.rxPos now points at the first firmware byte
        line.trim();
        return line.length() > 0;
      }
      if (c != '\r') line += c;
      if (line.length() > 200) return false;  // runaway guard
    }
  }
}

static long otaParseToken(const String& src, const char* key, long fallback) {
  int p = src.indexOf(key);
  if (p == -1) return fallback;
  p += strlen(key);
  int end = p;
  while (end < (int)src.length() && src[end] != ' ' && src[end] != '\r' && src[end] != '\n') end++;
  return src.substring(p, end).toInt();
}

static String otaParseStr(const String& src, const char* key) {
  int p = src.indexOf(key);
  if (p == -1) return String("");
  p += strlen(key);
  int end = p;
  while (end < (int)src.length() && src[end] != ' ' && src[end] != '\r' && src[end] != '\n') end++;
  return src.substring(p, end);
}

// Returns true only when an update was applied (in which case it reboots and
// never returns). Returns false for "no update" or any failure (caller keeps
// running the current firmware normally).
static bool runOtaUpdate(void) {
  Serial.printf("[OTA] check start (running fw v%d)\n", (int)FW_VERSION);
  oled_status("OTA", "checking...", -1);

  if (!moduleInitialized) initializeModule();
  if (!moduleInitialized) {
    Serial.println("[OTA] modem not registered, skip");
    return false;
  }

  const int contextID = 1;
  const int connectID = 0;
  const int accessMode = 0;  // buffer access mode -> AT+QIRD retrieves received bytes
  String openCmd = String("AT+QIOPEN=") + String(contextID) + "," + String(connectID) +
                   ",\"TCP\",\"" + OTA_SERVER_IP + "\"," + String(OTA_TCP_PORT) + ",0," + String(accessMode);
  String resp = sendATCommand(openCmd, 10000);
  if (resp.indexOf("OK") == -1) {
    Serial.println("[OTA] QIOPEN issue failed");
    return false;
  }
  if (!waitForQIOPEN(connectID, 60000, resp)) {
    Serial.println("[OTA] QIOPEN timeout");
    otaCloseSocket(connectID);
    return false;
  }

  OtaSock s;
  s.id = connectID;
  s.rxLen = 0;
  s.rxPos = 0;

  String req = String("OTA_CHECK device_id=") + String((unsigned long long)device_id, HEX) +
               " ver=" + String((int)FW_VERSION) + "\n";
  if (!sendBinaryOverTCP(connectID, (const uint8_t*)req.c_str(), req.length(), 5000, 10000)) {
    Serial.println("[OTA] request send failed");
    otaCloseSocket(connectID);
    return false;
  }
  delay(2000);  // let SEND OK finish and server begin reply (NB-IoT RTT)

  String header;
  if (!otaReadHeaderLine(connectID, s, header, 60000)) {
    Serial.println("[OTA] no header from server");
    otaCloseSocket(connectID);
    return false;
  }
  Serial.println("[OTA] server: " + header);

  if (header.startsWith("OTA_NONE")) {
    Serial.println("[OTA] up to date");
    oled_status("OTA", "up to date", -1);
    otaCloseSocket(connectID);
    return false;
  }
  if (!header.startsWith("OTA_BEGIN")) {
    Serial.println("[OTA] unexpected header");
    otaCloseSocket(connectID);
    return false;
  }

  long newSize = otaParseToken(header, "size=", 0);
  long newVer = otaParseToken(header, "ver=", 0);
  String md5hex = otaParseStr(header, "md5=");
  md5hex.toLowerCase();
  Serial.printf("[OTA] update available: size=%ld ver=%ld md5=%s\n", newSize, newVer, md5hex.c_str());
  if (newSize <= 0) {
    otaCloseSocket(connectID);
    return false;
  }

  const esp_partition_t* update = esp_ota_get_next_update_partition(NULL);
  if (update == NULL) {
    Serial.println("[OTA] no OTA partition (flash a build with the OTA partition table first)");
    oled_status("OTA", "no ota part", -1);
    otaCloseSocket(connectID);
    return false;
  }
  if (newSize > (long)update->size) {
    Serial.printf("[OTA] image (%ld) larger than slot (%u)\n", newSize, (unsigned)update->size);
    otaCloseSocket(connectID);
    return false;
  }
  Serial.printf("[OTA] writing to partition '%s' @0x%x\n", update->label, (unsigned)update->address);

  esp_ota_handle_t otaHandle = 0;
  esp_err_t err = esp_ota_begin(update, newSize, &otaHandle);
  if (err != ESP_OK) {
    Serial.printf("[OTA] esp_ota_begin failed: %s\n", esp_err_to_name(err));
    otaCloseSocket(connectID);
    return false;
  }

  md5_context_t md5ctx;
  esp_rom_md5_init(&md5ctx);

  static uint8_t dlbuf[512];
  long received = 0;
  unsigned long lastLog = millis();
  unsigned long dlStart = millis();
  bool ok = true;
  while (received < newSize) {
    if (millis() - dlStart > OTA_DOWNLOAD_BUDGET_MS) {
      Serial.println("[OTA] aborted: download budget exceeded");
      ok = false;
      break;
    }
    int want = (newSize - received < (long)sizeof(dlbuf)) ? (int)(newSize - received) : (int)sizeof(dlbuf);
    if (!otaReadExact(s, dlbuf, want, OTA_READ_CHUNK_TIMEOUT_MS)) {
      Serial.printf("[OTA] download stalled at %ld/%ld\n", received, newSize);
      ok = false;
      break;
    }
    if (esp_ota_write(otaHandle, dlbuf, want) != ESP_OK) {
      Serial.println("[OTA] esp_ota_write failed");
      ok = false;
      break;
    }
    esp_rom_md5_update(&md5ctx, dlbuf, want);
    received += want;
    if (millis() - lastLog > 3000) {
      lastLog = millis();
      int pct = (int)(received * 100 / newSize);
      Serial.printf("[OTA] %ld/%ld (%d%%)\n", received, newSize, pct);
      char m[20];
      snprintf(m, sizeof(m), "%d%% %ldk", pct, received / 1024);
      oled_status("OTA dl", m, pct);
    }
  }

  unsigned char digest[16];
  esp_rom_md5_final(digest, &md5ctx);
  otaCloseSocket(connectID);

  if (!ok) {
    esp_ota_abort(otaHandle);
    oled_status("OTA", "dl failed", -1);
    return false;
  }

  String gotMd5;
  char hx[3];
  for (int i = 0; i < 16; i++) {
    snprintf(hx, sizeof(hx), "%02x", digest[i]);
    gotMd5 += hx;
  }
  if (md5hex.length() == 32 && gotMd5 != md5hex) {
    Serial.println("[OTA] MD5 mismatch got=" + gotMd5 + " exp=" + md5hex);
    esp_ota_abort(otaHandle);
    oled_status("OTA", "MD5 FAIL", -1);
    return false;
  }

  err = esp_ota_end(otaHandle);
  if (err != ESP_OK) {
    Serial.printf("[OTA] esp_ota_end (image validate) failed: %s\n", esp_err_to_name(err));
    oled_status("OTA", "img invalid", -1);
    return false;
  }
  err = esp_ota_set_boot_partition(update);
  if (err != ESP_OK) {
    Serial.printf("[OTA] set_boot_partition failed: %s\n", esp_err_to_name(err));
    oled_status("OTA", "setboot fail", -1);
    return false;
  }

  Serial.printf("[OTA] SUCCESS: v%d -> v%ld, rebooting into '%s'\n",
                (int)FW_VERSION, newVer, update->label);
  oled_status("OTA OK", "reboot", 100);
  delay(1500);
  esp_restart();
  return true;  // unreachable
}

static bool sendPhotoOverTCP(const uint8_t* buffer, size_t length) {
  Serial.println("sendPhotoOverTCP() starting...");
  oled_status("Starting TCP...", "", 0);
  generateMeasurementId();
  Serial.printf("Measurement ID: %s\n", measurement_id);

  if (!moduleInitialized) {
    oled_layout_transmit("Init module...");
    initializeModule();
  }
  oled_layout_transmit("Open TCP conn...");
  int contextID = 1;
  int connectID = 0;
  // Buffer access mode (0): incoming data is held in the modem buffer and read
  // with AT+QIRD — which is what receiveFromTCP() uses. Mode 1 (direct push)
  // made the server's set_config reply arrive as an unsolicited URC nothing
  // was reading, so every reply-wait timed out. (The OTA socket already used 0.)
  int accessMode = 0;
  String openCmd = String("AT+QIOPEN=") + String(contextID) + "," + String(connectID) +
                   ",\"TCP\",\"" + SERVER_IP + "\"," + String(TCP_PORT) + ",0," + String(accessMode);
  String response = sendATCommand(openCmd, 10000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to issue QIOPEN!");
    oled_layout_transmit("TCP OPEN ERR");
    return false;
  }
  if (!waitForQIOPEN(connectID, 60000, response)) {
    oled_layout_transmit("TCP TIMEOUT");
    return false;
  }
  Serial.println("TCP connection opened.");
  oled_status("TCP Connected", "", 5);

  oled_status("Sending telemetry", "", 10);
  String lineProtocolASCII = buildLineProtocolASCII();
  String lineProtocolHex = asciiToHex(lineProtocolASCII);
  String header = "CORRELATION_HEADER\n";
  header += "app:container-classifier\n";
  header += "device_id:" + String((unsigned long long)device_id, HEX) + "\n";
  header += "measurement_id:" + String(measurement_id) + "\n";
  header += "timestamp:" + String(millis()) + "\n";
  header += "image_size:" + String(length) + "\n";
  header += "telemetry:" + lineProtocolHex + "\n";
  header += "END_HEADER\n";
  if (!sendBinaryOverTCP(connectID, (const uint8_t*)header.c_str(), header.length(), 5000, 10000)) {
    Serial.println("Header SEND FAIL");
    oled_status("TELEMETRY FAIL", "", -1);
    return false;
  }
  Serial.println("Correlation header sent.");
  delay(200);

  const size_t CHUNK_SIZE = 1400;
  size_t offset = 0;
  int totalChunks = (length + CHUNK_SIZE - 1) / CHUNK_SIZE;
  char imageInfoStr[24];
  snprintf(imageInfoStr, sizeof(imageInfoStr), "Image %dx%d", oledStatus.imageWidth, oledStatus.imageHeight);

  const uint8_t BGN_MARKER[] = {0x00, 0x00, 0x00, 0x00};
  if (!sendBinaryOverTCP(connectID, BGN_MARKER, sizeof(BGN_MARKER), 5000, 5000)) {
    oled_status("IMAGE FAIL", "Marker error", -1);
    return false;
  }

  while (offset < length) {
    size_t thisChunkSize = (length - offset) < CHUNK_SIZE ? (length - offset) : CHUNK_SIZE;
    int currentChunk = (int)(offset / CHUNK_SIZE) + 1;
    bool sent = false;
    for (int attempt = 0; attempt < 5 && !sent; attempt++) {
      sent = sendBinaryOverTCP(connectID, buffer + offset, thisChunkSize, 5000, 10000);
      if (!sent) delay(500 * (attempt + 1));
    }
    if (!sent) {
      Serial.printf("Failed to send chunk %d\n", currentChunk);
      char failChunkStr[24];
      snprintf(failChunkStr, sizeof(failChunkStr), "Chunk %d", currentChunk);
      oled_status("IMAGE FAIL", failChunkStr, -1);
      sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
      return false;
    }
    int chunkPercent = 15 + (currentChunk * 80 / totalChunks);
    char chunkInfo[24];
    snprintf(chunkInfo, sizeof(chunkInfo), "Chunk %d/%d", currentChunk, totalChunks);
    oled_status(imageInfoStr, chunkInfo, chunkPercent);
    offset += thisChunkSize;
    // Inter-chunk pacing. Do NOT shorten this below ~250ms: with fast pacing
    // (v3 used 10ms) several TCP segments are in flight at once, and under
    // radio loss the modem's TCP stack has been observed DUPLICATING a block
    // of payload during retransmission — the server then receives more bytes
    // than image_size and the JPEG is corrupt mid-stream (no images stored
    // 2026-06-18 → 07-02). ~250ms lets each segment get radio-acked before
    // the next is queued, keeping ≤1 segment in flight. Costs ~6s per photo
    // on a device that sleeps for an hour.
    if (length - offset < (CHUNK_SIZE * 3)) delay(300);
    else delay(250);
  }
  delay(500);

  const uint8_t END_MARKER[] = {0xFF, 0xFF, 0xFF, 0xFF};
  (void)sendBinaryOverTCP(connectID, END_MARKER, sizeof(END_MARKER), 5000, 10000);
  delay(500);

  applyTcpServerReply(connectID);

  oled_status(imageInfoStr, "Closing", 98);
  sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
  oled_status("TX Complete!", "", 100);
  return true;
}

// Phase 2b: full runMainJob
static void runMainJob(void) {
  // Flash marker: unambiguous confirmation of which build is running. Bump
  // FW_VERSION in platformio.ini on every firmware change so this is meaningful.
  Serial.printf("[FW] build fw v%d (%s %s)\n", (int)FW_VERSION, __DATE__, __TIME__);
  if (isCycleTimeoutExceeded()) {
    Serial.println("SAFETY: runMainJob aborted - global timeout exceeded");
    applyDaytimeSleepFromGeofence();
    enterDaytimeSleepMode();
    return;
  }
  readBatteryStatus();
  Serial.println("Starting main job - BC95 modem already initialized.");
#if BENCH_SKIP_UPLINK
  Serial.println("[BENCH] SKIP_UPLINK: skipping modem registration / telemetry / OTA");
#else
  initializeModule();
#endif

#if HAS_GNSS
  oled_status("GNSS...", "acquire", 0);
  if (g_gnssLastFix.valid) {
    Serial.printf("[GNSS] Reusing early fix: lat=%.7f lon=%.7f sats=%d\n",
                  g_gnssLastFix.lat_deg, g_gnssLastFix.lon_deg, g_gnssLastFix.sats_used);
    oled_status("GNSS", "OK", 5);
  } else {
    GnssFix fix;
    bool gotFix = gnssReadLatestFixSinceWake(fix, GNSS_ACQUIRE_TIMEOUT_MS, gnssSearchAnimTick);
    g_gnssLastFix = fix;
    if (gotFix) {
      Serial.printf("[GNSS] Fix: lat=%.7f lon=%.7f alt=%.2fm sats=%d hdop=%.2f\n",
                    g_gnssLastFix.lat_deg, g_gnssLastFix.lon_deg, g_gnssLastFix.altitude_m,
                    g_gnssLastFix.sats_used, g_gnssLastFix.hdop);
      oled_status("GNSS", "OK", 5);
    } else {
      Serial.println("[GNSS] No valid fix within timeout");
      oled_status("GNSS", "NO FIX", 5);
    }
  }
#else
  g_gnssLastFix = GnssFix{};
  Serial.println("[GNSS] Disabled (HW variant)");
#endif

#if !BENCH_SKIP_UPLINK
  updateOledSignalFromCSQ();
  int netHour = -1, netMin = -1;
  String rawClock;
  if (queryNetworkTime(netHour, netMin, rawClock)) {
    oledStatus.networkHour = netHour;
    oledStatus.networkMinute = netMin;
  }
  oled_update();

  if (doCameraTCPJob) {
    if (classificationPhoto != nullptr && classificationPhotoSize > 0) {
      Serial.println("Using classification photo for transmission");
      oled_layout_transmit("Send photo...");
      bool sendSuccess = sendPhotoOverTCP(classificationPhoto, classificationPhotoSize);
      heap_caps_free(classificationPhoto);
      classificationPhoto = nullptr;
      classificationPhotoSize = 0;
      if (sendSuccess) {
        Serial.println("Classification photo transmitted successfully");
        oled_layout_transmit("PHOTO SENT OK");
      } else {
        Serial.println("WARNING: Photo transmission FAILED");
        oled_layout_transmit("PHOTO TX FAIL");
      }
    } else {
      Serial.println("No classification photo available");
      oled_layout_transmit("NO PHOTO");
    }
  }

#if OTA_VIA_SERVER_COMMAND
  // Server asked us to update (via {"cmd":"ota"} in the telemetry reply). The
  // telemetry socket is closed by now, so it is safe to open the OTA socket.
  if (g_otaRequested) {
    g_otaRequested = false;
    runOtaUpdate();  // reboots into the new image on success; returns otherwise
  }
#endif
#if OTA_CHECK_ON_COLD_BOOT
  // Cold-boot OTA runs here (after telemetry) when the modem has a proven TCP path.
  if (g_otaColdBootPending) {
    g_otaColdBootPending = false;
    Serial.println("[OTA] post-job update check");
    runOtaUpdate();
  }
#endif
#endif  // !BENCH_SKIP_UPLINK
}

void runMultiImpulseClassification(uint8_t* image_buffer) {
  (void)image_buffer;
  ei_impulse_result_t result = {0};
  ei_signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;
  signal.total_length = impulse_785968_1.dsp_input_frame_size;
  signal.get_data = &get_signal_data_785968;
  EI_IMPULSE_ERROR res = process_impulse(&impulse_handle_785968_1, &signal, &result, debug_nn);
  if (res == EI_IMPULSE_OK) {
    float max_confidence = 0.0f;
    int best_class = -1;
    for (uint16_t i = 0; i < impulse_785968_1.label_count; i++) {
      if (result.classification[i].value > max_confidence) {
        max_confidence = result.classification[i].value;
        best_class = i;
      }
    }
    if (best_class >= 0) {
      levelConfidence = max_confidence;
      if (strstr(impulse_785968_1.categories[best_class], "level_0") != nullptr) fillLevel = 0.0f;
      else if (strstr(impulse_785968_1.categories[best_class], "level_1") != nullptr) fillLevel = 0.25f;
      else if (strstr(impulse_785968_1.categories[best_class], "level_2") != nullptr) fillLevel = 0.5f;
      else if (strstr(impulse_785968_1.categories[best_class], "level_3") != nullptr) fillLevel = 0.75f;
      else if (strstr(impulse_785968_1.categories[best_class], "level_4") != nullptr) fillLevel = 1.0f;
      else fillLevel = -1.0f;
    }
  }
  signal.total_length = impulse_785971_1.dsp_input_frame_size;
  signal.get_data = &get_signal_data_785971;
  res = process_impulse(&impulse_handle_785971_1, &signal, &result, debug_nn);
  if (res == EI_IMPULSE_OK) {
    for (uint16_t i = 0; i < impulse_785971_1.label_count && i < 13; i++) {
      materialClassConfidences[i] = result.classification[i].value;
    }
    float max_confidence = 0.0f;
    int best_class = -1;
    for (uint16_t i = 0; i < impulse_785971_1.label_count; i++) {
      if (result.classification[i].value > max_confidence) {
        max_confidence = result.classification[i].value;
        best_class = i;
      }
    }
    if (best_class >= 0) {
      detectedMaterial = String(impulse_785971_1.categories[best_class]);
      materialConfidence = max_confidence;
    }
  }
  overallConfidence = (levelConfidence + materialConfidence) / 2.0f;
  oledStatus.fillLevel = fillLevel >= 0 ? fillLevel * 100.0f : -1.0f;
  oledStatus.material = detectedMaterial;
  oledStatus.lvlConf = levelConfidence;
  oledStatus.matConf = materialConfidence;
  oled_update();
  oled_layout_main();

  // Reuse the level-model result computed in the first pass above. Previously the
  // level model was re-invoked here just to obtain max-confidence, which doubled
  // per-frame inference cost (~150-500 ms and associated PSRAM traffic) for no
  // new information. `levelConfidence` is already the max over label_count.
  if (levelConfidence >= 0.3f) {
    ei_printf("*** HIGH CONFIDENCE - running main job and sleeping ***\r\n");
    runMainJob();
    applyDaytimeSleepFromGeofence();
    enterDaytimeSleepMode();
  }
}

#define APP_TASK_STACK 10240
static void appTask(void* pvParameters) {
  // If the camera never came up this wake, don't spin retrying capture (which
  // just logs "Camera is not initialized") for the full cycle timeout — that
  // burns ~2 min of full-power time. Sleep now and retry on the next wake.
  if (!is_initialised) {
    ei_printf("Camera unavailable this wake — skipping capture, sleeping to retry\n");
    applyDaytimeSleepFromGeofence();
    enterDaytimeSleepMode();
    return;
  }
  for (;;) {
    if (isCycleTimeoutExceeded()) {
      ei_printf("SAFETY: cycle timeout, entering sleep\n");
      applyDaytimeSleepFromGeofence();
      enterDaytimeSleepMode();
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    snapshot_buf = (uint8_t*)heap_caps_malloc(
        EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE,
        MALLOC_CAP_SPIRAM);
    if (snapshot_buf == nullptr) {
      snapshot_buf = (uint8_t*)malloc(
          EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    }
    if (snapshot_buf == nullptr) {
      ei_printf("ERR: Failed to allocate snapshot buffer!\n");
      continue;
    }
    if (!ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
      heap_caps_free(snapshot_buf);
      snapshot_buf = nullptr;
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    runMultiImpulseClassification(snapshot_buf);
    if (snapshot_buf) {
      heap_caps_free(snapshot_buf);
      snapshot_buf = nullptr;
    }
  }
}

static void deIsolatePeripheralPinsOnWake(void) {
  // Undo RTC isolation set before deep sleep.  Do NOT call gpio_reset_pin() here:
  // that would re-enable default internal pull-ups while the camera rail (LD_SWCH_1)
  // is still OFF, back-feeding the camera through I/O protection diodes and leaving
  // it latched in an undefined state.  After this function, all pins are de-isolated
  // but still Hi-Z; caller must set them explicitly before powering the rail.
  const int pins[] = {
      SIOD_GPIO_NUM, SIOC_GPIO_NUM,
      VSYNC_GPIO_NUM, HREF_GPIO_NUM, XCLK_GPIO_NUM, PCLK_GPIO_NUM,
      Y2_GPIO_NUM, Y3_GPIO_NUM, Y4_GPIO_NUM, Y5_GPIO_NUM,
      Y6_GPIO_NUM, Y7_GPIO_NUM, Y8_GPIO_NUM, Y9_GPIO_NUM,
      BC95_RX_PIN, BC95_TX_PIN,
  };

  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    int g = pins[i];
    if (g < 0) continue;
    if (rtc_gpio_is_valid_gpio((gpio_num_t)g)) {
      rtc_gpio_deinit((gpio_num_t)g);
    }
  }
}

void setup() {
  // Phase 1a: DFR0975 order — brownout off, load switches on, then Serial
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Release all GPIO holds set before deep sleep. This MUST mirror the hold set in
  // enterSleepMode() (holdPinLowForSleep on SDA/SCL/XCLK + all camera DVP pins).
  // Per-pin gpio_hold_en() latches survive a warm reset (esp_restart, USB-CDC reset)
  // and even deep-sleep wake until explicitly cleared; if PCLK/VSYNC/HREF/Y2..Y9 are
  // left latched LOW, the sensor inits but streams 0 frames. Only a true power-on
  // cleared them, which is why only cold boot worked. Runs unconditionally.
  gpio_deep_sleep_hold_dis();
  const int heldPins[] = {LD_SWCH_1, LD_SWCH_2,
                          SIOD_GPIO_NUM, SIOC_GPIO_NUM, XCLK_GPIO_NUM,
                          PCLK_GPIO_NUM, VSYNC_GPIO_NUM, HREF_GPIO_NUM,
                          Y2_GPIO_NUM, Y3_GPIO_NUM, Y4_GPIO_NUM, Y5_GPIO_NUM,
                          Y6_GPIO_NUM, Y7_GPIO_NUM, Y8_GPIO_NUM, Y9_GPIO_NUM};
  for (size_t i = 0; i < sizeof(heldPins) / sizeof(heldPins[0]); i++) {
    int g = heldPins[i];
    if (g < 0) continue;
    if (rtc_gpio_is_valid_gpio((gpio_num_t)g)) {
      rtc_gpio_hold_dis((gpio_num_t)g);
      rtc_gpio_deinit((gpio_num_t)g);
    } else {
      gpio_hold_dis((gpio_num_t)g);
    }
  }

  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
#if PIR_WAKE_ENABLE
  const bool motionWake = isMotionExtWakeCause(wakeupCause);
#else
  const bool motionWake = false;
#endif
  bool fromDeepSleep = (wakeupCause == ESP_SLEEP_WAKEUP_TIMER ||
                        wakeupCause == ESP_SLEEP_WAKEUP_EXT0 ||
                        wakeupCause == ESP_SLEEP_WAKEUP_EXT1);

  // #region agent log — debug wake sequence
  Serial.begin(115200); delay(50);
  // Read LD_SWCH_1 state BEFORE we touch it — tells us if hold kept it LOW during sleep
  int ldswch1_raw = digitalRead(LD_SWCH_1);
  Serial.printf("[DBG-59d7ad] wake=%d fromDeepSleep=%d motionWake=%d LD_SWCH_1_raw=%d\n",
                (int)wakeupCause, fromDeepSleep, motionWake, ldswch1_raw);
  // #endregion

#if PIR_WAKE_ENABLE
  if (motionWake) {
    handleSpuriousPirWakeAndResleep();
  }
#endif

  // Step 1: Undo RTC isolation (no pull-ups yet — rail is still off).
  // MUST run on every boot, not just deep-sleep wakes. rtc_gpio_isolate() set in
  // enterSleepMode() lives in the always-on RTC domain and survives a warm reset
  // (esp_restart, USB-CDC reset, brownout). If skipped there, the camera's DVP pins
  // (PCLK/VSYNC/data) stay isolated -> sensor inits but streams 0 frames. A true
  // power-on clears the RTC domain, which is why only cold boot worked. Harmless on
  // cold boot: rtc_gpio_deinit() on non-isolated pins is a no-op.
  deIsolatePeripheralPinsOnWake();
  // #region agent log
  Serial.printf("[DBG-59d7ad] RTC de-isolated (%s)\n", fromDeepSleep ? "deep-sleep wake" : "warm/cold boot");
  // #endregion

  // Step 2: All camera/I2C pins to Hi-Z with no pulls BEFORE powering the rail.
  // This prevents back-feeding the unpowered camera through internal pull-ups.
  prepareVddDomainPinsForSleep();
  highz_no_pulls(SIOD_GPIO_NUM);
  highz_no_pulls(SIOC_GPIO_NUM);
  highz_no_pulls(BC95_RX_PIN);
  highz_no_pulls(BC95_TX_PIN);
  // #region agent log
  Serial.println("[DBG-59d7ad] All VDD-domain pins Hi-Z, no pulls");
  // #endregion

  // Step 3: Power-cycle the load switches for a clean camera power-on reset.
  pinMode(LD_SWCH_1, OUTPUT);
  pinMode(LD_SWCH_2, OUTPUT);
  digitalWrite(LD_SWCH_1, LOW);
  digitalWrite(LD_SWCH_2, LOW);
  Serial.printf("[DBG-59d7ad] LD_SWCH levels now: sw1=%d sw2=%d\n", digitalRead(LD_SWCH_1), digitalRead(LD_SWCH_2));

  // While rails are OFF, actively pull SCCB lines LOW to discharge any residual charge
  // and force a clean power-on reset of the camera/sensors (pull-ups are on the switched rail).
  pinMode(SIOD_GPIO_NUM, OUTPUT);
  pinMode(SIOC_GPIO_NUM, OUTPUT);
  digitalWrite(SIOD_GPIO_NUM, LOW);
  digitalWrite(SIOC_GPIO_NUM, LOW);
  pinMode(XCLK_GPIO_NUM, OUTPUT);
  digitalWrite(XCLK_GPIO_NUM, LOW);

  // #region agent log
  Serial.println("[DBG-59d7ad] LD_SWCH LOW — rails off, discharging SCCB/XCLK");
  // #endregion

  // Give the power switch time to fully turn off and peripherals time to brown out.
  // Longer on a deep-sleep wake: the camera rail must fully discharge for a clean POR.
  delay(fromDeepSleep ? 2000 : 500);

  // Power rails back up while SDA/SCL/XCLK are still driven LOW, then release — keeps
  // I2C pull-ups from back-feeding the camera through its protection diodes during rise.
  digitalWrite(LD_SWCH_1, HIGH);
  digitalWrite(LD_SWCH_2, HIGH);
  delay(50);
  highz_no_pulls(SIOD_GPIO_NUM);
  highz_no_pulls(SIOC_GPIO_NUM);
  highz_no_pulls(XCLK_GPIO_NUM);
  Serial.printf("[DBG-59d7ad] LD_SWCH levels after HIGH: sw1=%d sw2=%d\n", digitalRead(LD_SWCH_1), digitalRead(LD_SWCH_2));
  // #region agent log
  Serial.printf("[DBG-59d7ad] LD_SWCH HIGH — waiting 2500ms (t=%lu)\n", millis());
  // #endregion
  delay(2500);

  // USB CDC settle after rail change (Serial already begun at start of setup).
  delay(500);
  {
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {
      delay(10);
    }
  }

  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_POWERON) {
    Serial.println("=== COLD BOOT ===");
  } else if (wakeupCause == ESP_SLEEP_WAKEUP_TIMER || wakeupCause == ESP_SLEEP_WAKEUP_EXT0 || wakeupCause == ESP_SLEEP_WAKEUP_EXT1) {
#if PIR_WAKE_ENABLE
    if (motionWake) {
      Serial.println("=== DEEP SLEEP WAKEUP (PIR / EXT) ===");
    } else {
      Serial.println("=== DEEP SLEEP WAKEUP (timer) ===");
    }
#else
    Serial.println("=== DEEP SLEEP WAKEUP ===");
#endif
    if (!motionWake) {
      customSleepDuration = DEFAULT_SLEEP_TIME;
    }
  } else {
    Serial.println("=== WARM RESET ===");
  }
  Serial.println("Load switches ON (GPIO 13, 14)");
#if HAS_GNSS && GNSS_POWER_PERMANENT && GNSS_EXTINT_WAKE
  // Release any deep-sleep latch on the rewired EXTINT line and hold it idle LOW. The actual
  // wake pulse happens later in gnssWakeFromBackup() when the first DDC read finds the
  // receiver still in standby.
  gnssExtintInit();
#endif
  heap_caps_malloc_extmem_enable(100000);

  // Modem + NVS first: night schedule gate runs before camera/OLED so timer/PIR wake
  // outside the daytime window does one long deep sleep until WAKE (PIR disabled there)
  // — no camera warm-up, GNSS, or ML.
  Serial.printf("Initializing BC95 modem (RX: GPIO%d, TX: GPIO%d, baud: %d)...\n",
                BC95_RX_PIN, BC95_TX_PIN, (int)BC95_BAUD);
  if (resetReason == ESP_RST_POWERON) {
    // Cold boot: open at target baud, auto-detect + migrate if still at fallback.
    if (!modemBootAndSetBaud(30, 500)) {
      Serial.println("WARNING: NB-IoT module may not be fully booted");
    }
  } else if (wakeupCause == ESP_SLEEP_WAKEUP_TIMER || wakeupCause == ESP_SLEEP_WAKEUP_EXT0 || wakeupCause == ESP_SLEEP_WAKEUP_EXT1) {
    // Modem stayed powered through PSM, so it keeps its UART baud. Open at the
    // target; if it doesn't answer, fall back + migrate.
    bc95_modem.begin(BC95_BAUD, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
    if (usePSM) {
      wakeModuleFromPSM();
    } else {
      delay(500);
    }
#if BC95_BAUD != BC95_BAUD_FALLBACK
    if (!modemRespondsAT()) {
      bc95_modem.begin(BC95_BAUD_FALLBACK, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
      delay(40);
      if (modemRespondsAT()) raiseModemBaudToTarget();
      else bc95_modem.begin(BC95_BAUD, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
    }
#endif
  } else {
    bc95_modem.begin(BC95_BAUD, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
    delay(500);
  }
  Serial.println("BC95 modem serial initialized.");

  initializeDeviceIdentity();
  loadSleepGeoFromNvs();

#if OTA_CHECK_ON_COLD_BOOT
  // True power-on only (not deep-sleep wakes): check for a firmware update before
  // the night-sleep gate so the device can be updated even outside operating
  // hours. Reboots into the new image on success; otherwise continues boot.
  if (resetReason == ESP_RST_POWERON) {
    Serial.println("[OTA] cold boot -> deferred until after main job");
    g_otaColdBootPending = true;
  }
#endif

  tryScheduleNightSleepIfOutsideWindow();

  // Step 4: Reset I2C controller state before camera/OLED use.
  Wire.end();
  i2c_bus_initialized = false;
  pinMode(SIOD_GPIO_NUM, INPUT);
  pinMode(SIOC_GPIO_NUM, INPUT);
  delay(50);

  // Step 5 (wake only): Avoid gpio_reset_pin() on GPIO1/GPIO2 (shared SCCB/I2C lines).
  // On ESP32-S3 those pins can have special boot/USB roles and resetting them here has
  // caused intermittent SCCB probe failures after deep sleep.
  // Instead, perform a hard I2C bus recovery *after* rails are up, then leave pin
  // configuration to esp_camera_init().
  {
    // Temporarily enable internal pull-ups (rail is ON now) to help the bus reach a clean HIGH
    // even if external pull-ups are weak.
    pinMode(SIOD_GPIO_NUM, INPUT_PULLUP);
    pinMode(SIOC_GPIO_NUM, INPUT_PULLUP);
    delay(5);
    i2cBusRecover();
    // Back to Hi-Z/no-pulls; camera driver will configure SCCB.
    highz_no_pulls(SIOD_GPIO_NUM);
    highz_no_pulls(SIOC_GPIO_NUM);
  }

  Serial.printf("[DBG-59d7ad] I2C reset+recovery done, ready for camera init (t=%lu)\n", millis());

  // Depot / no-fix: sleep before camera warm-up, ML, and upload. Outside: full loop below.
  tryEarlyGeofenceSleepRouting();

  // Phase 1b: camera init with retry
  bool cameraOk = false;
  const int MAX_CAMERA_RETRIES = 3;
  for (int attempt = 1; attempt <= MAX_CAMERA_RETRIES; attempt++) {
    Serial.printf("Camera init attempt %d/%d...\n", attempt, MAX_CAMERA_RETRIES);
    // Refresh the OLED so it doesn't sit on the GNSS screen through the camera
    // attempts. Draw now (bus is free here); camera_init re-takes the bus after.
    if (s_oled_initialized) {
      Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
      Wire.setClock(I2C_CLOCK_HZ);
      char cm[20];
      snprintf(cm, sizeof(cm), "init %d/%d", attempt, MAX_CAMERA_RETRIES);
      oled_status("Camera", cm, -1);
    }
    if (camera_init()) {
      cameraOk = true;
      break;
    }
    if (attempt < MAX_CAMERA_RETRIES) {
      Serial.println("Cleaning up and HW-resetting camera (LD_SWCH_1 cycle)...");
      esp_camera_deinit();
      s_camera_initialised = false;
      is_initialised = false;
      forceStopXclk();
      // Drive SDA/SCL/XCLK LOW to prevent back-feed, then cut power
      prepareVddDomainPinsForSleep();
      pinMode(SIOD_GPIO_NUM, OUTPUT); digitalWrite(SIOD_GPIO_NUM, LOW);
      pinMode(SIOC_GPIO_NUM, OUTPUT); digitalWrite(SIOC_GPIO_NUM, LOW);
      pinMode(XCLK_GPIO_NUM, OUTPUT); digitalWrite(XCLK_GPIO_NUM, LOW);
      digitalWrite(LD_SWCH_1, LOW);
      delay(2000);                       // longer discharge for a clean camera POR
      // Power the camera rail back up while SDA/SCL/XCLK are still driven LOW, so the
      // LD_SWCH_2 I2C pull-ups (OLED/fuel gauge stay powered) cannot back-feed the
      // sensor during the rail rise. Release the lines only after the rail is up.
      digitalWrite(LD_SWCH_1, HIGH);
      delay(50);
      highz_no_pulls(SIOD_GPIO_NUM);
      highz_no_pulls(SIOC_GPIO_NUM);
      highz_no_pulls(XCLK_GPIO_NUM);
      delay(2500);
    }
  }
  if (!cameraOk) {
    Serial.printf("ERROR: Camera failed after %d attempts\n", MAX_CAMERA_RETRIES);
  }

  // Phase 1c: I2C and OLED (after camera; same GPIO1/2 for I2C)
  // esp_camera_init() brings up SCCB (I2C) internally, so we take ownership of the bus here.
  // This avoids "Bus already started" warnings and reduces the chance of stale I2C state.
  Wire.end();
  i2c_bus_initialized = false;
  delay(10);

  Serial.printf("Initializing SSD1315 OLED (I2C SDA: GPIO%d, SCL: GPIO%d)...\n", SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  initI2CBusOnce();
  if (!u8g2.begin()) {
    Serial.println("ERROR: OLED display initialization failed!");
  } else {
    Serial.println("OLED display initialized successfully!");
    s_oled_initialized = true;
    // Shorter here: the main loading animation already ran early (pre-GNSS).
    oled_boot_loading(800);
  }
  oled_status("Starting...", "Init done", 0);
  if (useFuelGauge) {
    if (initializeMAX17048()) {
      Serial.println("MAX17048 OK");
    } else {
      Serial.println("MAX17048 init failed");
    }
  }

  // Modem already initialized (schedule gate + TCP). BC95 module remains available.

  s_serialMutex = xSemaphoreCreateMutex();
  if (s_serialMutex != NULL) {
    Serial.println("=== ESP32-S3 FreeRTOS app starting ===");
  }

  cycleStartTime = millis();
  xTaskCreate(appTask, "app", APP_TASK_STACK, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
