#pragma once
// BLE link to the Mac: Nordic-UART-style GATT service.
//   TX characteristic (notify): binary frames, see FRAME FORMAT in ble_link.cpp and tools/ble_bridge.py
//   RX characteristic (write) : text commands ("reset", "calib", "set txHz 50", ...)
#include <Arduino.h>
#include "estimator.h"

struct RawSample { int16_t v[6]; };  // ax ay az (LSB, 4096/g), gx gy gz (LSB, 65.5/dps)

void bleBegin(const char* name, void (*onCmd)(const char*));
bool bleConnected();
uint16_t blePayloadMax();               // largest frame that fits one notification (MTU - 3)
bool bleCanSend();                      // a notification buffer is free (flow control)
bool bleSendPose(const PoseMsg& p);
bool bleSendPothole(const PotholeMsg& h);
bool bleSendText(uint8_t type, const char* s);  // FRAME_STATUS (JSON) or FRAME_INFO (text, chunked)
bool bleSendRaw(uint32_t tMsFirst, const RawSample* s, uint8_t n);
int  bleRawPerFrame();
uint32_t bleConnects();
uint32_t bleNotifyErrors();
uint32_t bleFramesSent();

enum : uint8_t { FRAME_POSE = 1, FRAME_POTHOLE = 2, FRAME_STATUS = 3, FRAME_INFO = 4, FRAME_RAW = 5 };
