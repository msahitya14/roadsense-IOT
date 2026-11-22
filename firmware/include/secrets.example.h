#pragma once
// Copy to secrets.h and fill in (or run broker/setup.sh, which writes it for a local broker).
// secrets.h is git-ignored: never commit real passwords.

#define WIFI_SSID   "your-hotspot-name"      // 2.4 GHz only (ESP32 has no 5 GHz radio)
#define WIFI_PASS   "your-hotspot-password"

// ---------------- Option 1: local Mosquitto on the laptop (broker/run.sh) ----------------
#define MQTT_HOST   "192.168.1.50"           // laptop IP on the same Wi-Fi (run.sh prints/updates it)
#define MQTT_PORT   1883
#define MQTT_TLS    0
#define MQTT_USER   "rs01"
#define MQTT_PASS   "change-me"

// ---------------- Option 2: cloud broker over TLS (e.g. HiveMQ Cloud free tier) ------------
// #define MQTT_HOST   "xxxxxxxxxxxx.s1.eu.hivemq.cloud"
// #define MQTT_PORT   8883
// #define MQTT_TLS    1
// #define MQTT_USER   "rs01"                 // create this user in the HiveMQ Cloud console
// #define MQTT_PASS   "change-me"
// Optional but recommended: pin the root CA so the ESP32 verifies the broker.
// HiveMQ Cloud uses Let's Encrypt -> ISRG Root X1 (https://letsencrypt.org/certs/isrgrootx1.pem).
// #define MQTT_CA_CERT "-----BEGIN CERTIFICATE-----\n" \
//                      "...paste each line of the PEM here, ending with \\n ...\n" \
//                      "-----END CERTIFICATE-----\n"

// ---------------- AP mode (NET_MODE 1/2 in config.h) -----------------------------------------
// #define AP_PASS "roadsense24"                // >= 8 characters
