#pragma once
#include <Arduino.h>
#include "estimator.h"

// Network health, written by loop(), shown on the NET page.
struct NetState {
  bool wifi, mqtt, ap;
  int rssi, apClients;
  char ip[16], ssid[24], broker[16];
  uint32_t queued, dropped, reconnects, imuErr, mqttErr;
};

// Hooks the UI task uses to reach the rest of the firmware.
struct UiHooks {
  Estimator* est;
  void (*getNet)(NetState&);
  void (*sendCmd)(const char*);  // e.g. "reset", "calib"
};

bool uiBegin();                                   // OLED + LEDs + buttons; false if no OLED
void uiLines(const char* l1, const char* l2 = "", const char* l3 = "", const char* l4 = "");  // setup() only
void uiNotify(const String& msg, uint16_t ms = 2500);  // banner on the bottom line (any task)
void uiStart(const UiHooks& hooks);               // starts the UI task
