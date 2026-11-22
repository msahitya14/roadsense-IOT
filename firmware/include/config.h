#pragma once
// ---------------------------------------------------------------------------
// RoadSense build-time configuration. Values that need tuning on the car
// (speed, gyro scale, thresholds) are NOT here: they are runtime parameters,
// changed over serial / MQTT with `set <key> <value>` and saved to flash (NVS).
// ---------------------------------------------------------------------------

#define DEVICE_ID      "rs01"
#define TOPIC_BASE     "rs/T24"          // team 24 -> rs/T24/pose, rs/T24/pothole, ...

// Pins come from the course kit pin map (include/iot_pins.h, copied from the labs).
#include "iot_pins.h"
#define I2C_SDA        IoTPins::I2C_SDA       // 21
#define I2C_SCL        IoTPins::I2C_SCL       // 22
#define MPU_ADDR       IoTPins::MPU6050_ADDR  // 0x68 (0x69 is tried automatically)

// Kit peripherals used for on-car debugging (set to 0 if not wired)
#define USE_OLED       1   // SPI SSD1306 128x64: CS 5, DC 16, RST 17, SCK 18, MOSI 23
#define USE_LEDS       1   // red 2 = pothole flash, yellow 4 = not connected/buffering, green 15 = driving
#define USE_BUTTONS    1   // BTN1 (32) = next OLED page, BTN2 (33) = reset pose, hold BTN2 2 s = calib
#define USE_BUZZER     1   // short beep on pothole (27)
#define OLED_FPS       5

#define SAMPLE_HZ      200               // IMU sample + estimator rate
#define POSE_PERIOD_S  1                 // pose/surface message every N seconds

// Offline buffer: 600 pose messages = 10 min at 1 Hz
#define POSE_QUEUE_LEN     600
#define POTHOLE_QUEUE_LEN  50

// Networking
//   0 = join the hotspot in secrets.h only
//   1 = always access-point mode: the ESP32 creates Wi-Fi "RoadSense-rs01" and the laptop joins it
//   2 = auto: try the hotspot, fall back to AP mode if it never connects within AP_FALLBACK_S
// In modes 1/2, holding BTN1 while powering on also forces AP mode.
// AP password defaults to "roadsense24"; override with #define AP_PASS in secrets.h.
#define NET_MODE       0   // 0 for a cloud broker / hotspot; 1 for the offline AP demo
#define AP_FALLBACK_S  20

// Accept commands on rs/T24/cmd/<dev> (handy for calibration from the dashboard).
// STRIDE "E" says no inbound command channel in production: set to 0 for the final build.
#define ALLOW_MQTT_CMDS 1
