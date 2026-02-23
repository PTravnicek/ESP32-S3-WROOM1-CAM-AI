#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "mbedtls/sha256.h"
#include <WiFi.h>
#include <DFRobot_SHT3x.h>
#include "DFRobot_GNSS.h"
#include <ArduinoJson.h>

#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "model-parameters/model_variables.h"

#define VDD_POWER 13  // Camera power GPIO

// Camera pins (DFR0975)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define SIOD_GPIO_NUM    1
#define SIOC_GPIO_NUM    2
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    42
#define XCLK_GPIO_NUM    45
#define PCLK_GPIO_NUM    5
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      46
#define Y7_GPIO_NUM      8
#define Y6_GPIO_NUM      7
#define Y5_GPIO_NUM      4
#define Y4_GPIO_NUM      41
#define Y3_GPIO_NUM      40
#define Y2_GPIO_NUM      39

#define WARMUP_TARGET_FRAMES 30
#define WARMUP_TIMEOUT_MS    10000
#define MAX_CONSECUTIVE_NULL_FRAMES 5
#define WARMUP_FRAME_DELAY_MS 50

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

#define BC95_RX_PIN 44  // ESP32 RX connected to BC95 TX
#define BC95_TX_PIN 43  // ESP32 TX connected to BC95 RX
HardwareSerial bc95_modem(2);
static const char* SERVER_IP = "34.10.203.180";
static const int TCP_PORT = 8009;
static bool moduleInitialized = false;

// TCP receive: wait for server response after sending data
#define TCP_RECV_TIMEOUT_MS 15000   // How long to wait for server response
#define TCP_RECV_POLL_MS    1000    // Poll interval for AT+QIRD
#define TCP_RECV_BUFFER_SIZE 512    // Max command size from server

static bool usePSM = false;  // Set true to keep modem in PSM during sleep (VDD must stay HIGH)
#define PSM_ACTIVE_TIME "00100001"   // 1 minute
#define PSM_PERIODIC_TAU "00100001"  // 1 hour
static bool useEDRX = true;
#define EDRX_CYCLE "0011"  // 40.96 seconds

static uint64_t device_id = 0;
static char measurement_id[37] = {0};

// Telemetry stubs for buildLineProtocolASCII (sensors can be added later)
static int batteryPercentage = 0;
static bool sensorDataValid = false;
static struct { float temperature = 0.0f; float humidity = 0.0f; } sensorData;
static bool gpsFixObtained = false;
static bool gnssCoordinatesValid = false;
static double gnssLatitude = 0.0;
static double gnssLongitude = 0.0;

static bool useGNSS = false;  // Enable GNSS module (GPS + GLONASS)
DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);

static bool useSHT30 = true;
static bool useFuelGauge = true;
#define MAX17048_I2C_ADDR 0x36
static DFRobot_SHT3x sht30(&Wire, 0x45);

// GNSS: timeout for fix acquisition and NVS fallback
#define TIMEOUT_DURATION 120000   // ms (2 min) for getGNSS loop
#define NVS_GNSS_NAMESPACE "gnss_pos"
#define NVS_KEY_LAT "lat"
#define NVS_KEY_LON "lon"
#define NVS_KEY_TS "ts"
#define GNSS_MAX_STALE_AGE_SEC (24 * 60 * 60)  // 24 hours
static bool gnssPositionIsStale = false;

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

// GNSS NVS: save/load last-known position for fallback
static void saveGNSSToNVS(double lat, double lon) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_GNSS_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    Serial.printf("Failed to open GNSS NVS namespace: %s\n", esp_err_to_name(err));
    return;
  }
  int64_t lat_int = (int64_t)(lat * 1e7);
  int64_t lon_int = (int64_t)(lon * 1e7);
  uint32_t timestamp = (uint32_t)(millis() / 1000);
  nvs_set_i64(h, NVS_KEY_LAT, lat_int);
  nvs_set_i64(h, NVS_KEY_LON, lon_int);
  nvs_set_u32(h, NVS_KEY_TS, timestamp);
  err = nvs_commit(h);
  nvs_close(h);
  if (err == ESP_OK) {
    Serial.printf("Saved GNSS to NVS: %.6f, %.6f (ts=%u)\n", lat, lon, timestamp);
  } else {
    Serial.printf("Failed to commit GNSS to NVS: %s\n", esp_err_to_name(err));
  }
}

static bool loadGNSSFromNVS(double& lat, double& lon, uint32_t& age_sec) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_GNSS_NAMESPACE, NVS_READONLY, &h);
  if (err != ESP_OK) return false;
  int64_t lat_int = 0, lon_int = 0;
  uint32_t saved_ts = 0;
  err = nvs_get_i64(h, NVS_KEY_LAT, &lat_int);
  if (err != ESP_OK) { nvs_close(h); return false; }
  err = nvs_get_i64(h, NVS_KEY_LON, &lon_int);
  if (err != ESP_OK) { nvs_close(h); return false; }
  nvs_get_u32(h, NVS_KEY_TS, &saved_ts);
  nvs_close(h);
  lat = (double)lat_int / 1e7;
  lon = (double)lon_int / 1e7;
  uint32_t current_ts = (uint32_t)(millis() / 1000);
  age_sec = (current_ts > saved_ts) ? (current_ts - saved_ts) : 0;
  Serial.printf("Loaded GNSS from NVS: %.6f, %.6f (age ~%u s)\n", lat, lon, (unsigned)age_sec);
  return true;
}

// OLED: SSD1315 compatible with SSD1306 driver, 128x64, HW I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static bool showOLED = true;

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

static void oled_splash(void) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 10, " _     _                 _");
  u8g2.drawStr(0, 18, "(_)___| |_  _ __  __ _ _(_)__");
  u8g2.drawStr(0, 26, "| / _ \\  _|| '  \\/ _` / _` / _|");
  u8g2.drawStr(0, 34, "|_\\___/\\__||_|_|_\\__,_\\__, \\__|");
  u8g2.drawStr(0, 42, "                      |___/");
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(30, 58, "Container AI v1.2");
  u8g2.sendBuffer();
}

static void getGNSS(void) {
  unsigned long startTime = millis();
  gnssPositionIsStale = false;
  while (millis() - startTime < (unsigned long)TIMEOUT_DURATION) {
    if (isCycleTimeoutExceeded()) {
      Serial.println("GNSS: Global cycle timeout, aborting");
      break;
    }
    uint8_t numSatellites = gnss.getNumSatUsed();
    if (numSatellites > 0) {
      sLonLat_t lat = gnss.getLat();
      sLonLat_t lon = gnss.getLon();
      oled_status("GNSS", "fix OK", -1);
      gnssLatitude = lat.latitudeDegree;
      if ((char)lat.latDirection == 'S') gnssLatitude = -gnssLatitude;
      gnssLongitude = lon.lonitudeDegree;
      if ((char)lon.lonDirection == 'W') gnssLongitude = -gnssLongitude;
      gnssCoordinatesValid = true;
      gnssPositionIsStale = false;
      gpsFixObtained = true;
      saveGNSSToNVS(gnssLatitude, gnssLongitude);
      return;
    }
    delay(5000);
    char elapsed[16];
    snprintf(elapsed, sizeof(elapsed), "%lu s", (unsigned long)((millis() - startTime) / 1000));
    oled_status("GNSS time", elapsed, -1);
  }
  Serial.println("GNSS timeout - attempting NVS fallback...");
  double fallbackLat, fallbackLon;
  uint32_t age_sec;
  if (loadGNSSFromNVS(fallbackLat, fallbackLon, age_sec) && age_sec < GNSS_MAX_STALE_AGE_SEC) {
    gnssLatitude = fallbackLat;
    gnssLongitude = fallbackLon;
    gnssCoordinatesValid = true;
    gnssPositionIsStale = true;
    gpsFixObtained = false;
    oled_status("GNSS", "NVS fallback", -1);
    return;
  }
  oled_status("GNSS", "timeout no fix", -1);
  gpsFixObtained = false;
  gnssCoordinatesValid = false;
  gnssPositionIsStale = false;
}

// Mutex so Serial output is not interleaved (Serial is not thread-safe)
static SemaphoreHandle_t s_serialMutex = NULL;

static camera_config_t s_camera_config;
static bool s_camera_initialised = false;

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
    s->set_vflip(s, 1);
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
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }
  is_initialised = false;
  s_camera_initialised = false;
}

static void configurePSM(void);
static void prepareModuleForPSM(void);
static void wakeModuleFromPSM(void);

// Phase 2b: full enterSleepMode (PSM optional; timer + VDD off)
static void enterSleepMode(void) {
  Serial.println("Preparing for sleep...");

  if (usePSM) {
    Serial.println("Configuring NB-IoT PSM...");
    configurePSM();
    prepareModuleForPSM();
  }

  // 1) Stop radios
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  // 2) Deinit camera
  Serial.println("Deinitializing camera...");
  ei_camera_deinit();
  delay(100);

  // 3) Sleep timer
  esp_sleep_enable_timer_wakeup((uint64_t)customSleepDuration * uS_TO_S_FACTOR);
  Serial.printf("Setup ESP32 to sleep for %d seconds\n", customSleepDuration);

  // 4) OLED sleep message
  char sleepMsg[24];
  snprintf(sleepMsg, sizeof(sleepMsg), "Sleep %ds", customSleepDuration);
  oled_status("SLEEPING...", sleepMsg, -1);

  // 5) VDD_POWER off (camera and peripherals)
  Serial.println("Turning VDD_POWER OFF");
  digitalWrite(VDD_POWER, LOW);
  rtc_gpio_hold_en((gpio_num_t)VDD_POWER);

  // 6) Free classification photo
  if (classificationPhoto != nullptr) {
    heap_caps_free(classificationPhoto);
    classificationPhoto = nullptr;
    classificationPhotoSize = 0;
    Serial.println("Cleaned up classification photo");
  }

  delay(100);
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
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

/** Update oledStatus.signalRSSI from AT+CSQ (0-31 or 99). */
static void updateOledSignalFromCSQ(void) {
  String csqResponse = sendATCommand("AT+CSQ", 1000);
  int csqPos = csqResponse.indexOf("+CSQ:");
  if (csqPos != -1) {
    int commaPos = csqResponse.indexOf(',', csqPos);
    if (commaPos != -1) {
      String rssiStr = csqResponse.substring(csqPos + 5, commaPos);
      rssiStr.trim();
      int rssi = rssiStr.toInt();
      oledStatus.signalRSSI = rssi;
    }
  }
}

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
  String psmStatus = sendATCommand("AT+CPSMS?", 1000);
  if (psmStatus.indexOf("1,") == -1 && psmStatus.indexOf(": 1") == -1) {
    String psmCmd = String("AT+CPSMS=1,,,\"") + PSM_PERIODIC_TAU + "\",\"" + PSM_ACTIVE_TIME + "\"";
    sendATCommand(psmCmd, 1000);
  }
  String sclkStatus = sendATCommand("AT+QSCLK?", 500);
  if (sclkStatus.indexOf("1") == -1) {
    sendATCommand("AT+QSCLK=1", 500);
  }
  oled_status("PSM ready", "3.3uA", -1);
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

  Serial.println("Rebooting module...");
  oled_status("module reboot", "wait", -1);
  sendATCommand("AT&F0", 1000);
  Serial.println("Configuring APN (iot.1nce.net)...");
  sendATCommand("AT+QCGDEFCONT=\"IP\",\"iot.1nce.net\"", 2000);
  sendATCommand("AT+NRB", 1000);
  Serial.println("Waiting for module reboot...");
  delay(4000);
  sendATCommand("AT", 500);
  sendATCommand("ATE0", 500);
  sendATCommand("AT+QSCLK=0", 500);
  Serial.println("Setting CFUN=1...");
  oled_status("set CFUN", "1", -1);
  sendATCommand("AT+CFUN=1", 3000);
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

  String CEREGresponse;
  int ceregChecks = 0;
  const int MAX_CEREG_CHECKS = 150;
  bool registered = false;
  do {
    ceregChecks++;
    CEREGresponse = sendATCommand("AT+CEREG?", 2000);
    if (CEREGresponse.indexOf(",1") != -1 && CEREGresponse.indexOf("+CEREG:") != -1) {
      int ceregPos = CEREGresponse.indexOf("+CEREG:");
      int commaPos = CEREGresponse.indexOf(',', ceregPos);
      if (commaPos != -1 && CEREGresponse.charAt(commaPos + 1) == '1') {
        registered = true;
        oled_status("network home", "OK", -1);
        break;
      }
    }
    if (CEREGresponse.indexOf(",5") != -1 && CEREGresponse.indexOf("+CEREG:") != -1) {
      int ceregPos = CEREGresponse.indexOf("+CEREG:");
      int commaPos = CEREGresponse.indexOf(',', ceregPos);
      if (commaPos != -1 && CEREGresponse.charAt(commaPos + 1) == '5') {
        registered = true;
        oled_status("network roam", "OK", -1);
        break;
      }
    }
    delay(2000);
  } while (!registered && ceregChecks < MAX_CEREG_CHECKS);

  if (registered) {
    Serial.println("Network registered successfully!");
    String cgpaddrVerify = sendATCommand("AT+CGPADDR", 1000);
  } else {
    Serial.println("ERROR: Failed to register on network");
    oled_status("network", "error timeout", -1);
  }
  String cgattResponse = sendATCommand("AT+CGATT?", 1000);
  moduleInitialized = registered;
  Serial.printf("Module init complete. Status: %s\n", moduleInitialized ? "OK" : "Failed");
}

// Phase 2b: optional sensors (SHT30, MAX17048)
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
static void readSHT30Sensor(void) {
  if (!useSHT30) return;
  float t = sht30.getTemperatureC();
  float h = sht30.getHumidityRH();
  if (isnan(t) || isnan(h)) {
    sensorDataValid = false;
    return;
  }
  if (h < 0.0f) h = 0.0f;
  if (h > 100.0f) h = 100.0f;
  sensorData.temperature = t;
  sensorData.humidity = h;
  sensorDataValid = true;
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

static String bufferToHex(const uint8_t* data, size_t length) {
  String hex;
  hex.reserve(length * 2);
  for (size_t i = 0; i < length; i++) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02X", data[i]);
    hex += tmp;
  }
  return hex;
}

static String buildLineProtocolASCII(void) {
  String lineProtocol;
  lineProtocol.reserve(200);
  lineProtocol = "gps_data,device=esp32 ";
  lineProtocol += "fix=" + String(gpsFixObtained ? 1 : 0) + ",";
  lineProtocol += "temp=" + String(sensorDataValid ? sensorData.temperature : NAN, 1) + ",";
  lineProtocol += "hum=" + String(sensorDataValid ? sensorData.humidity : NAN, 1) + ",";
  lineProtocol += "lat=" + String(gnssCoordinatesValid ? gnssLatitude : NAN, 6) + ",";
  lineProtocol += "lon=" + String(gnssCoordinatesValid ? gnssLongitude : NAN, 6) + ",";
  lineProtocol += "bat=" + String(batteryPercentage) + ",";
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
    int newSleep = doc["value"] | DEFAULT_SLEEP_TIME;
    customSleepDuration = newSleep;
    Serial.printf("Command: set_sleep=%d\n", newSleep);
    char msg[24];
    snprintf(msg, sizeof(msg), "sleep=%d s", newSleep);
    oled_status("CMD Received", msg, -1);
    delay(2000);
  } else if (strcmp(cmd, "ack") == 0) {
    Serial.println("Command: ack (no change)");
    oled_status("CMD Received", "ACK", -1);
    delay(1000);
  } else {
    Serial.printf("Unknown command: %s\n", cmd);
    oled_status("CMD Unknown", cmd, -1);
    delay(1000);
  }
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
  int accessMode = 1;
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
  header += "device_id:" + String((unsigned long long)device_id, HEX) + "\n";
  header += "measurement_id:" + String(measurement_id) + "\n";
  header += "timestamp:" + String(millis()) + "\n";
  header += "image_size:" + String(length) + "\n";
  header += "telemetry:" + lineProtocolHex + "\n";
  header += "END_HEADER\n";
  String headerHex = asciiToHex(header);
  String headerCmd = String("AT+QISENDEX=") + String(connectID) + "," + String(headerHex.length() / 2) + ",\"" + headerHex + "\"";
  String headerResponse = sendATCommand(headerCmd, 5000);
  if (!waitForSendResult(10000, headerResponse)) {
    Serial.println("Header SEND FAIL");
    oled_status("TELEMETRY FAIL", "", -1);
    return false;
  }
  Serial.println("Correlation header sent.");
  delay(1000);

  const size_t CHUNK_SIZE = 1400;
  size_t offset = 0;
  int totalChunks = (length + CHUNK_SIZE - 1) / CHUNK_SIZE;
  char imageInfoStr[24];
  snprintf(imageInfoStr, sizeof(imageInfoStr), "Image %dx%d", oledStatus.imageWidth, oledStatus.imageHeight);

  const uint8_t BGN_MARKER[] = {0x00, 0x00, 0x00, 0x00};
  String bgnMarkerHex = bufferToHex(BGN_MARKER, 4);
  String bgnCmd = String("AT+QISENDEX=") + String(connectID) + ",4,\"" + bgnMarkerHex + "\"";
  String bgnResponse = sendATCommand(bgnCmd, 1000);
  if (!waitForSendResult(5000, bgnResponse)) {
    oled_status("IMAGE FAIL", "Marker error", -1);
    return false;
  }

  while (offset < length) {
    size_t thisChunkSize = (length - offset) < CHUNK_SIZE ? (length - offset) : CHUNK_SIZE;
    int currentChunk = (int)(offset / CHUNK_SIZE) + 1;
    String hexChunk = bufferToHex(buffer + offset, thisChunkSize);
    String sendCmd = String("AT+QISENDEX=") + String(connectID) + "," + String(thisChunkSize) + ",\"" + hexChunk + "\"";
    bool sent = false;
    for (int attempt = 0; attempt < 5 && !sent; attempt++) {
      modemFlushInput(5);
      String chunkResponse = sendATCommand(sendCmd, 2000);
      sent = waitForSendResult(10000, chunkResponse);
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
    if (offset >= length) delay(3000);
    else if (length - offset < (CHUNK_SIZE * 3)) delay(500);
    else delay(20);
  }
  delay(5000);

  const uint8_t END_MARKER[] = {0xFF, 0xFF, 0xFF, 0xFF};
  String endMarkerHex = bufferToHex(END_MARKER, 4);
  String endCmd = String("AT+QISENDEX=") + String(connectID) + ",4,\"" + endMarkerHex + "\"";
  String endResponse = sendATCommand(endCmd, 5000);
  waitForSendResult(10000, endResponse);
  delay(2000);

  // Wait for server response (bidirectional communication)
  oled_status("Waiting for", "server...", -1);
  String serverResponse;
  if (receiveFromTCP(connectID, serverResponse, TCP_RECV_TIMEOUT_MS)) {
    Serial.println("Server response: " + serverResponse);
    parseAndApplyCommand(serverResponse);
  } else {
    Serial.println("No response from server (timeout)");
    oled_status("No server", "response", -1);
    delay(1000);
  }

  oled_status(imageInfoStr, "Closing", 98);
  sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
  oled_status("TX Complete!", "", 100);
  return true;
}

// Phase 2b: full runMainJob
static void runMainJob(void) {
  if (isCycleTimeoutExceeded()) {
    Serial.println("SAFETY: runMainJob aborted - global timeout exceeded");
    enterSleepMode();
    return;
  }
  readSHT30Sensor();
  readBatteryStatus();
  Serial.println("Starting main job - BC95 modem already initialized.");
  initializeModule();

  if (useGNSS) getGNSS();

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
}

void checkHighConfidenceAndSleep(ei_impulse_result_t* result) {
  float maxConfidence = 0.0f;
  bool foundHighConfidence = false;
  for (uint16_t i = 0; i < impulse_785968_1.label_count; i++) {
    if (result->classification[i].value >= 0.3f && result->classification[i].value > maxConfidence) {
      maxConfidence = result->classification[i].value;
      foundHighConfidence = true;
    }
  }
  if (foundHighConfidence) {
    ei_printf("*** HIGH CONFIDENCE - sending stub and sleeping 10s ***\r\n");
    runMainJob();
    enterSleepMode();
  }
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
  signal.total_length = impulse_785968_1.dsp_input_frame_size;
  signal.get_data = &get_signal_data_785968;
  res = process_impulse(&impulse_handle_785968_1, &signal, &result, false);
  if (res == EI_IMPULSE_OK) {
    float max_confidence = 0.0f;
    for (uint16_t i = 0; i < impulse_785968_1.label_count; i++) {
      if (result.classification[i].value > max_confidence) max_confidence = result.classification[i].value;
    }
    if (max_confidence >= 0.3f) {
      checkHighConfidenceAndSleep(&result);
    }
  }
}

#define APP_TASK_STACK 10240
static void appTask(void* pvParameters) {
  for (;;) {
    if (isCycleTimeoutExceeded()) {
      ei_printf("SAFETY: cycle timeout, entering sleep\n");
      enterSleepMode();
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

void setup() {
  // Phase 1a: DFR0975 order — brownout off, camera power, then Serial
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  rtc_gpio_hold_dis((gpio_num_t)VDD_POWER);
  pinMode(VDD_POWER, OUTPUT);
  digitalWrite(VDD_POWER, HIGH);

  delay(300);

  Serial.begin(115200);
  delay(500);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) {
    delay(10);
  }

  // Optional: boot detection and PSRAM
  esp_reset_reason_t resetReason = esp_reset_reason();
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  if (resetReason == ESP_RST_POWERON) {
    Serial.println("=== COLD BOOT ===");
  } else if (wakeupCause == ESP_SLEEP_WAKEUP_TIMER || wakeupCause == ESP_SLEEP_WAKEUP_EXT0 || wakeupCause == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("=== DEEP SLEEP WAKEUP ===");
    customSleepDuration = DEFAULT_SLEEP_TIME;
  } else {
    Serial.println("=== WARM RESET ===");
  }
  Serial.println("Camera power ON (GPIO 13)");
  heap_caps_malloc_extmem_enable(100000);

  // Phase 1b: camera init with retry
  bool cameraOk = false;
  const int MAX_CAMERA_RETRIES = 3;
  for (int attempt = 1; attempt <= MAX_CAMERA_RETRIES; attempt++) {
    Serial.printf("Camera init attempt %d/%d...\n", attempt, MAX_CAMERA_RETRIES);
    if (camera_init()) {
      cameraOk = true;
      break;
    }
    if (attempt < MAX_CAMERA_RETRIES) {
      Serial.println("Cleaning up and waiting 2s before retry...");
      esp_camera_deinit();
      s_camera_initialised = false;
      is_initialised = false;
      delay(2000);
    }
  }
  if (!cameraOk) {
    Serial.printf("ERROR: Camera failed after %d attempts\n", MAX_CAMERA_RETRIES);
  }

  // Phase 1c: I2C and OLED (after camera; same GPIO1/2 for I2C)
  Serial.printf("Initializing SSD1315 OLED (I2C SDA: GPIO%d, SCL: GPIO%d)...\n", SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  initI2CBusOnce();
  if (!u8g2.begin()) {
    Serial.println("ERROR: OLED display initialization failed!");
  } else {
    Serial.println("OLED display initialized successfully!");
    oled_splash();
    delay(2000);
  }
  oled_status("Starting...", "Init done", 0);

  if (useSHT30) {
    Serial.println("Initializing SHT30...");
    if (sht30.begin() == 0) {
      Serial.println("SHT30 OK");
    } else {
      Serial.println("SHT30 init failed");
    }
  }
  if (useFuelGauge) {
    if (initializeMAX17048()) {
      Serial.println("MAX17048 OK");
    } else {
      Serial.println("MAX17048 init failed");
    }
  }

  // Phase 2b: BC95 modem init
  Serial.printf("Initializing BC95 modem (RX: GPIO%d, TX: GPIO%d)...\n", BC95_RX_PIN, BC95_TX_PIN);
  bc95_modem.begin(9600, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  if (resetReason == ESP_RST_POWERON) {
    if (!waitForModuleBoot(30, 500)) {
      Serial.println("WARNING: NB-IoT module may not be fully booted");
    }
  } else if (wakeupCause == ESP_SLEEP_WAKEUP_TIMER || wakeupCause == ESP_SLEEP_WAKEUP_EXT0 || wakeupCause == ESP_SLEEP_WAKEUP_EXT1) {
    if (usePSM) {
      wakeModuleFromPSM();
    } else {
      delay(500);
    }
  } else {
    delay(500);
  }
  Serial.println("BC95 modem serial initialized.");

  initializeDeviceIdentity();

  if (useGNSS) {
    while (!gnss.begin()) {
      Serial.println("GNSS not found");
      oled_status("GNSS not", "found", -1);
      delay(1000);
    }
    gnss.enablePower();
    gnss.setGnss(eGPS_BeiDou_GLONASS);
    gnss.setRgbOff();
    getGNSS();
  }

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
