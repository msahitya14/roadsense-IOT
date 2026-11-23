// RoadSense: MPU-6050-only dead reckoning, surface and pothole logger on an RC car.
// Sensor + estimator run in their own FreeRTOS task at 200 Hz; Wi-Fi/MQTT runs in loop(),
// so a slow reconnect never stalls the integration. Pose messages wait in a 10-min queue.
#include <Arduino.h>
#include <WiFi.h>
#include <MQTT.h>
#include "config.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Missing include/secrets.h: run broker/setup.sh, or copy secrets.example.h to secrets.h and edit it"
#endif
#ifndef AP_SSID
#define AP_SSID "RoadSense-" DEVICE_ID
#endif
#ifndef AP_PASS
#define AP_PASS "roadsense24"  // >= 8 chars; override in secrets.h
#endif
#include "imu.h"
#include "estimator.h"
#include "debug_ui.h"
#include "ble_link.h"

struct CmdLine { char s[96]; };
struct InfoLine { char s[400]; };
struct RawItem { uint32_t tMs; RawSample s; };

static Estimator est;
static QueueHandle_t qPose, qPot, qCmd, qInfo, qRaw;
static volatile uint32_t droppedMsgs = 0, imuErrors = 0;
static volatile bool g_linkUp = false;      // data path to the laptop is up (BLE subscribed / MQTT connected)
static volatile float g_txRate = 0;         // pose frames actually sent per second
static const bool BLE_MODE = NET_MODE == 3;

#ifndef MQTT_TLS
#define MQTT_TLS 0
#endif
#if MQTT_TLS
#include <WiFiClientSecure.h>
static WiFiClientSecure net;  // cloud broker, e.g. HiveMQ Cloud on port 8883
#else
static WiFiClient net;        // local broker on port 1883
#endif
static MQTTClient mqtt(1024);
static char tPose[48], tPot[48], tStatus[48], tInfo[48], tCmd[64], willMsg[64];
static uint32_t reconnects = 0;
static int32_t lastMqttErr = 0;

// ---- access-point mode: the ESP32 makes its own Wi-Fi, the laptop joins it and runs the broker.
// The broker address is whatever IP the ESP32's DHCP server handed to a client (usually 192.168.4.2).
static bool apMode = false, staEverConnected = false;
static uint32_t staStartMs = 0;
static portMUX_TYPE apMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t apClientIps[4] = {0};  // most recent DHCP leases
static int apClientN = 0, apTry = 0;

static void onWifiEvent(WiFiEvent_t ev, WiFiEventInfo_t info) {
  if (ev == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED) {
    uint32_t ip = info.wifi_ap_staipassigned.ip.addr;
    portENTER_CRITICAL(&apMux);
    bool known = false;
    for (int i = 0; i < apClientN; i++) known |= apClientIps[i] == ip;
    if (!known) {
      for (int i = 3; i > 0; i--) apClientIps[i] = apClientIps[i - 1];  // newest first
      apClientIps[0] = ip;
      if (apClientN < 4) apClientN++;
      apTry = 0;
    }
    portEXIT_CRITICAL(&apMux);
  }
}

static bool linkUp() {
  return apMode ? (WiFi.softAPgetStationNum() > 0 && apClientN > 0) : WiFi.status() == WL_CONNECTED;
}

static void startAP();  // defined below (needs postInfo)

// network health for the OLED NET page
static NetState netState{};
static portMUX_TYPE netMux = portMUX_INITIALIZER_UNLOCKED;
static void getNet(NetState& out) {
  portENTER_CRITICAL(&netMux);
  out = netState;
  portEXIT_CRITICAL(&netMux);
}
static void updateNet() {
  NetState n{};
  if (BLE_MODE) {
    n.mode = 'B';
    n.wifi = n.mqtt = bleConnected();
    strlcpy(n.ssid, "RoadSense-" DEVICE_ID, sizeof(n.ssid));
    snprintf(n.ip, sizeof(n.ip), "mtu %u", (unsigned)(blePayloadMax() + 3));
    strlcpy(n.broker, "Mac bridge", sizeof(n.broker));
    n.txRate = g_txRate;
    n.bleErr = bleNotifyErrors();
    n.queued = uxQueueMessagesWaiting(qPose);
    n.dropped = droppedMsgs;
    n.reconnects = reconnects;
    n.imuErr = imuErrors;
    portENTER_CRITICAL(&netMux);
    netState = n;
    portEXIT_CRITICAL(&netMux);
    return;
  }
  n.mode = apMode ? 'A' : 'W';
  n.ap = apMode;
  n.wifi = linkUp();
  n.mqtt = n.wifi && mqtt.connected();
  n.rssi = (!apMode && n.wifi) ? WiFi.RSSI() : 0;
  n.apClients = apMode ? WiFi.softAPgetStationNum() : 0;
  strlcpy(n.ssid, apMode ? AP_SSID : WIFI_SSID, sizeof(n.ssid));
  strlcpy(n.ip, (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), sizeof(n.ip));
  strlcpy(n.broker, apMode ? (apClientN ? IPAddress(apClientIps[apTry % apClientN]).toString().c_str() : "wait laptop")
                           : MQTT_HOST, sizeof(n.broker));
  n.queued = uxQueueMessagesWaiting(qPose);
  n.dropped = droppedMsgs;
  n.reconnects = reconnects;
  n.imuErr = imuErrors;
  n.mqttErr = lastMqttErr;
  portENTER_CRITICAL(&netMux);
  netState = n;
  portEXIT_CRITICAL(&netMux);
}

static void postInfo(const String& s) {
  Serial.println(s);
  uiNotify(s.substring(0, 21));
  InfoLine l;
  strlcpy(l.s, s.c_str(), sizeof(l.s));
  if (xQueueSend(qInfo, &l, 0) != pdTRUE) {  // keep only the newest few
    InfoLine junk;
    xQueueReceive(qInfo, &junk, 0);
    xQueueSend(qInfo, &l, 0);
  }
}

template <typename T>
static void pushDropOldest(QueueHandle_t q, const T& m) {
  if (xQueueSend(q, &m, 0) != pdTRUE) {
    T junk;
    xQueueReceive(q, &junk, 0);
    droppedMsgs++;
    xQueueSend(q, &m, 0);
  }
}

// ------------------------------------------------------------------ sensor task
static void sensorTask(void*) {
  String msg;
  postInfo("Keep the car STILL: measuring gyro bias...");
  while (!est.calibrateStill(2000, msg)) { postInfo(msg); delay(500); }
  postInfo(msg);
  postInfo("Ready. Type `help` for commands.");

  TickType_t last = xTaskGetTickCount();
  uint32_t lastUs = micros(), n = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / SAMPLE_HZ));

    CmdLine c;
    while (xQueueReceive(qCmd, &c, 0) == pdTRUE) {
      postInfo(est.command(String(c.s)));
      last = xTaskGetTickCount();  // commands like calib block for a while
      lastUs = micros();
    }

    ImuSample s;
    if (!imuRead(s)) { imuErrors++; continue; }
    uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;
    if (dt <= 0 || dt > 0.05f) dt = 1.0f / SAMPLE_HZ;
    est.update(s, dt);

    if (BLE_MODE && est.rawLink && g_linkUp) {  // full-rate raw IMU, only while someone listens
      RawItem r{millis(), {{(int16_t)lroundf(s.ax * 4096), (int16_t)lroundf(s.ay * 4096), (int16_t)lroundf(s.az * 4096),
                            (int16_t)lroundf(s.gx * 65.5f), (int16_t)lroundf(s.gy * 65.5f), (int16_t)lroundf(s.gz * 65.5f)}}};
      xQueueSend(qRaw, &r, 0);  // dropped if the link can't keep up
    }

    PoseMsg p;
    static uint32_t lastBufMs = 0, lastPrintMs = 0;
    static bool lastBufMoving = false;
    if (est.poseReady(p)) {
      // online: every pose (up to txHz). offline: at most 1 Hz, so the queue holds ~15 min of driving
      if (g_linkUp || p.moving != lastBufMoving || millis() - lastBufMs >= 1000) {
        pushDropOldest(qPose, p);
        lastBufMs = millis();
        lastBufMoving = p.moving;
      }
      bool print = millis() - lastPrintMs >= 1000 || !p.moving;
      if (print) lastPrintMs = millis();
      if (est.streamHz == 0 && print)
        Serial.printf("pose x=%.2f y=%.2f hdg=%.1f v=%.2f d=%.2f vib=%.3f accSd=%.3f %s %s\n", p.x, p.y,
                      p.hdg, p.spd, p.dist, p.vib, p.accSd, p.moving ? "DRIVING" : "parked",
                      SURF_NAMES[p.surf]);
    }
    PotholeMsg h;
    if (est.potholeReady(h)) {
      pushDropOldest(qPot, h);
      Serial.printf("POTHOLE at (%.2f, %.2f) zneg=%.2f zpos=%.2f pitch=%.0f dps\n", h.x, h.y, h.zneg, h.zpos,
                    h.pitch);
    }
    if (est.streamHz > 0 && (n % max(1, SAMPLE_HZ / est.streamHz)) == 0)
      Serial.printf("R,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", (unsigned long)millis(), s.ax, s.ay, s.az, s.gx,
                    s.gy, s.gz);
    n++;
  }
}

static void sendCmd(const char* cmd) {  // used by the OLED buttons
  CmdLine c;
  strlcpy(c.s, cmd, sizeof(c.s));
  xQueueSend(qCmd, &c, 0);
}

// ------------------------------------------------------------------ network
static void onMqttMessage(String& topic, String& payload) {
#if ALLOW_MQTT_CMDS
  CmdLine c;
  strlcpy(c.s, payload.c_str(), sizeof(c.s));
  xQueueSend(qCmd, &c, 0);
#endif
}

static void publishStatus() {
  char b[256];
  snprintf(b, sizeof(b),
           "{\"dev\":\"%s\",\"state\":\"online\",\"ip\":\"%s\",\"rssi\":%d,\"up_s\":%lu,\"queued\":%u,"
           "\"dropped\":%lu,\"reconnects\":%lu,\"imu_err\":%lu}",
           DEVICE_ID, (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), apMode ? 0 : WiFi.RSSI(),
           (unsigned long)(millis() / 1000),
           (unsigned)uxQueueMessagesWaiting(qPose), (unsigned long)droppedMsgs, (unsigned long)reconnects,
           (unsigned long)imuErrors);
  mqtt.publish(tStatus, b, true, 1);
}

static void mqttConnect() {
  static uint32_t lastTry = 0;
  if (lastTry && millis() - lastTry < 3000) return;
  lastTry = millis();
  String host = MQTT_HOST;
  if (apMode) {  // broker = a laptop that joined our AP; rotate through the known clients
    portENTER_CRITICAL(&apMux);
    IPAddress ip(apClientIps[apTry % apClientN]);
    portEXIT_CRITICAL(&apMux);
    mqtt.setHost(ip, MQTT_PORT);
    host = ip.toString();
  }
  Serial.printf("MQTT: connecting to %s:%d as %s...\n", host.c_str(), MQTT_PORT, MQTT_USER);
  if (mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
    reconnects++;
    publishStatus();
#if ALLOW_MQTT_CMDS
    mqtt.subscribe(tCmd, 1);
#endif
    postInfo(String("MQTT connected, ") + uxQueueMessagesWaiting(qPose) + " buffered poses to send");
  } else {
    lastMqttErr = mqtt.returnCode() ? (int32_t)mqtt.returnCode() : (int32_t)mqtt.lastError();
    uiNotify(String("MQTT fail ") + lastMqttErr, 2000);
    if (apMode) apTry++;  // maybe that client was a phone, try the next one
    Serial.printf("MQTT connect failed (lastError %d, returnCode %d). Broker running? Same Wi-Fi? Firewall?\n",
                  mqtt.lastError(), mqtt.returnCode());
  }
}

static void drainQueues() {
  char b[400];
  for (int i = 0; i < 10; i++) {
    PotholeMsg h;
    if (xQueuePeek(qPot, &h, 0) != pdTRUE) break;
    snprintf(b, sizeof(b),
             "{\"dev\":\"%s\",\"ts\":%lu,\"seq\":%lu,\"x_m\":%.2f,\"y_m\":%.2f,\"z_neg_g\":%.2f,\"z_pos_g\":%.2f,"
             "\"pitch_dps\":%.0f}",
             DEVICE_ID, (unsigned long)h.ts, (unsigned long)h.seq, h.x, h.y, h.zneg, h.zpos, h.pitch);
    if (!mqtt.publish(tPot, b, false, 1)) return;
    xQueueReceive(qPot, &h, 0);
  }
  for (int i = 0; i < 20; i++) {
    PoseMsg p;
    if (xQueuePeek(qPose, &p, 0) != pdTRUE) break;
    snprintf(b, sizeof(b),
             "{\"dev\":\"%s\",\"ts\":%lu,\"seq\":%lu,\"x_m\":%.2f,\"y_m\":%.2f,\"hdg_deg\":%.1f,\"spd_mps\":%.2f,"
             "\"surf\":\"%s\",\"conf\":%.2f,\"dist_m\":%.2f,\"vib_g\":%.4f,\"acc_sd\":%.4f,\"mv\":%d,\"label\":\"%s\","
             "\"drv_s\":%.2f,\"yaw_raw\":%.2f,\"vib_int\":%.4f}",
             DEVICE_ID, (unsigned long)p.ts, (unsigned long)p.seq, p.x, p.y, p.hdg, p.spd, SURF_NAMES[p.surf],
             p.conf, p.dist, p.vib, p.accSd, p.moving, p.label, p.drvS, p.yawRaw, p.vibInt);
    if (!mqtt.publish(tPose, b, false, 1)) return;
    xQueueReceive(qPose, &p, 0);
  }
  InfoLine l;
  while (xQueueReceive(qInfo, &l, 0) == pdTRUE) mqtt.publish(tInfo, l.s, false, 0);
}

static void startAP() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 6 /*channel*/, 0 /*visible*/, 4 /*max clients*/);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.printf("softAP('%s') %s, ESP32 IP %s\n", AP_SSID, ok ? "started" : "FAILED",
                WiFi.softAPIP().toString().c_str());
  apMode = true;
  postInfo(String("AP MODE: join Wi-Fi '") + AP_SSID + "' pass '" + AP_PASS + "' on the laptop, then run broker/run.sh");
  uiNotify("AP " AP_SSID, 6000);
}

// ------------------------------------------------------------------ BLE transport
static void bleStatus() {
  char b[240];
  snprintf(b, sizeof(b),
           "{\"dev\":\"%s\",\"state\":\"online\",\"link\":\"ble\",\"mtu\":%u,\"up_s\":%lu,\"queued\":%u,"
           "\"dropped\":%lu,\"reconnects\":%lu,\"imu_err\":%lu,\"ble_err\":%lu,\"tx_hz\":%.1f}",
           DEVICE_ID, (unsigned)(blePayloadMax() + 3), (unsigned long)(millis() / 1000),
           (unsigned)uxQueueMessagesWaiting(qPose), (unsigned long)droppedMsgs, (unsigned long)reconnects,
           (unsigned long)imuErrors, (unsigned long)bleNotifyErrors(), (float)g_txRate);
  bleSendText(FRAME_STATUS, b);
}

static void bleService() {
  static bool wasUp = false;
  static uint32_t lastStatus = 0, rateT0 = 0, rateN = 0, lastAdvMsg = 0;
  bool up = bleConnected();
  if (up && !wasUp) {
    reconnects++;
    lastStatus = 0;
    postInfo(String("BLE connected, mtu ") + (blePayloadMax() + 3) + ", " + uxQueueMessagesWaiting(qPose) +
             " buffered poses");
  } else if (!up && wasUp) {
    Serial.println("BLE disconnected - buffering at 1 Hz");
  } else if (!up && millis() - lastAdvMsg > 5000) {
    lastAdvMsg = millis();
    Serial.printf("BLE: advertising as RoadSense-%s, waiting for the Mac bridge (%u poses buffered)\n", DEVICE_ID,
                  (unsigned)uxQueueMessagesWaiting(qPose));
  }
  wasUp = up;
  g_linkUp = up;
  if (millis() - rateT0 >= 1000) { g_txRate = rateN * 1000.0f / (millis() - rateT0); rateN = 0; rateT0 = millis(); }
  if (!up) return;

  // priority: potholes > poses > raw > info > status. Items leave a queue only once handed to the radio.
  PotholeMsg h;
  while (xQueuePeek(qPot, &h, 0) == pdTRUE && bleSendPothole(h)) xQueueReceive(qPot, &h, 0);
  PoseMsg p;
  for (int i = 0; i < 25 && xQueuePeek(qPose, &p, 0) == pdTRUE && bleSendPose(p); i++) {
    xQueueReceive(qPose, &p, 0);
    rateN++;
  }
  int per = bleRawPerFrame();
  RawItem first;
  while (uxQueueMessagesWaiting(qRaw) >= (UBaseType_t)per ||
         (xQueuePeek(qRaw, &first, 0) == pdTRUE && millis() - first.tMs > 50)) {
    if (!bleCanSend()) break;
    RawSample buf[20];
    uint32_t t0 = 0;
    uint8_t n = 0;
    RawItem r;
    while (n < per && xQueueReceive(qRaw, &r, 0) == pdTRUE) { if (!n) t0 = r.tMs; buf[n++] = r.s; }
    if (!n || !bleSendRaw(t0, buf, n)) break;
  }
  InfoLine l;
  if (bleCanSend() && xQueueReceive(qInfo, &l, 0) == pdTRUE) bleSendText(FRAME_INFO, l.s);
  DebugSnap d;
  est.snapshot(d);
  if (!lastStatus || millis() - lastStatus > (d.moving ? 5000UL : 30000UL)) {
    bleStatus();
    lastStatus = millis();
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== RoadSense (MPU-6050 only) ===");
  snprintf(tPose, sizeof(tPose), "%s/pose", TOPIC_BASE);
  snprintf(tPot, sizeof(tPot), "%s/pothole", TOPIC_BASE);
  snprintf(tStatus, sizeof(tStatus), "%s/status", TOPIC_BASE);
  snprintf(tInfo, sizeof(tInfo), "%s/info", TOPIC_BASE);
  snprintf(tCmd, sizeof(tCmd), "%s/cmd/%s", TOPIC_BASE, DEVICE_ID);
  snprintf(willMsg, sizeof(willMsg), "{\"dev\":\"%s\",\"state\":\"offline\"}", DEVICE_ID);

  uiBegin();
  uiLines("RoadSense " DEVICE_ID, "booting...", "looking for MPU-6050");

  // try both addresses at 400 kHz, then at 100 kHz (long wires / weak pull-ups)
  const uint8_t addrs[2] = {MPU_ADDR, MPU_ADDR == 0x68 ? 0x69 : 0x68};
  int who = -1;
  for (uint32_t hz : {400000u, 100000u}) {
    for (uint8_t a : addrs) {
      if (who < 0) {
        who = imuBegin(I2C_SDA, I2C_SCL, a, hz);
        if (who >= 0) Serial.printf("MPU-6050 at 0x%02X, I2C %lu kHz\n", a, (unsigned long)(hz / 1000));
      }
    }
  }
  while (who < 0) {
    String found = i2cScan();
    Serial.printf("MPU-6050 NOT FOUND on SDA=%d SCL=%d. I2C scan: %s\n", I2C_SDA, I2C_SCL, found.c_str());
    if (found == "none")
      Serial.println("  -> no device answers at all: check SDA/SCL not swapped, VCC->3V3, GND, loose jumpers");
    else
      Serial.println("  -> bus works (other parts answer) but no 0x68/0x69: MPU module power/solder/AD0 or a dead module");
    uiLines("MPU-6050 NOT FOUND", ("scan: " + found).c_str(), "SDA->21  SCL->22", "VCC->3V3 GND->GND");
    delay(2000);
    for (uint8_t a : addrs)
      if (who < 0) who = imuBegin(I2C_SDA, I2C_SCL, a, 100000);
  }
  Serial.printf("MPU-6050 found, WHO_AM_I=0x%02X\n", who);
  char whoLine[22];
  snprintf(whoLine, sizeof(whoLine), "MPU OK  WHO=0x%02X", who);
  uiLines("RoadSense " DEVICE_ID, whoLine, BLE_MODE ? "BLE: RoadSense-" DEVICE_ID : "Wi-Fi: " WIFI_SSID);

  est.begin();
  Serial.println("params " + est.paramsJson());

  qPose = xQueueCreate(POSE_QUEUE_LEN, sizeof(PoseMsg));
  qPot = xQueueCreate(POTHOLE_QUEUE_LEN, sizeof(PotholeMsg));
  qCmd = xQueueCreate(8, sizeof(CmdLine));
  qInfo = xQueueCreate(6, sizeof(InfoLine));

  qRaw = xQueueCreate(400, sizeof(RawItem));
  if (BLE_MODE) {
    bleBegin("RoadSense-" DEVICE_ID, sendCmd);
    Serial.println("BLE mode: advertising as RoadSense-" DEVICE_ID ". On the Mac: python tools/ble_bridge.py");
  }
  WiFi.onEvent(onWifiEvent);
  bool forceAP = NET_MODE == 1;
#if USE_BUTTONS
  if (NET_MODE != 0) forceAP |= digitalRead(IoTPins::BUTTON_1) == LOW;  // hold BTN1 at power-on -> AP mode
#endif
  if (BLE_MODE) {
    WiFi.mode(WIFI_OFF);
  } else if (forceAP) {
    startAP();
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    staStartMs = millis();
  }

#if MQTT_TLS
#ifdef MQTT_CA_CERT
  net.setCACert(MQTT_CA_CERT);  // verify the broker (STRIDE S/T)
#else
  net.setInsecure();            // encrypted, but the broker's certificate is NOT verified
  Serial.println("WARNING: MQTT_TLS without MQTT_CA_CERT -> server certificate not verified");
#endif
#endif
  mqtt.begin(MQTT_HOST, MQTT_PORT, net);
  mqtt.setKeepAlive(10);
  mqtt.setTimeout(MQTT_TLS ? 5000 : 2000);  // TLS handshake is slower
  mqtt.setWill(tStatus, willMsg, true, 1);
  mqtt.onMessage(onMqttMessage);

  updateNet();
  xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, nullptr, 3, nullptr, 1);
  uiStart({&est, getNet, sendCmd});
}

void loop() {
  // serial commands -> sensor task
  static char line[96];
  static size_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) {
        line[len] = 0;
        CmdLine c;
        strlcpy(c.s, line, sizeof(c.s));
        xQueueSend(qCmd, &c, 0);
        len = 0;
      }
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }

  static uint32_t lastNet = 0;
  if (millis() - lastNet > 250) { updateNet(); lastNet = millis(); }
  if (BLE_MODE) { bleService(); delay(2); return; }

  static bool ntpStarted = false;
  static uint32_t lastStatus = 0, lastWifiMsg = 0;
  // auto mode: hotspot never connected within AP_FALLBACK_S -> become an access point
  if (NET_MODE == 2 && !apMode && !staEverConnected && millis() - staStartMs > AP_FALLBACK_S * 1000UL) {
    Serial.printf("Wi-Fi '%s' not reachable after %d s -> switching to AP mode\n", WIFI_SSID, AP_FALLBACK_S);
    startAP();
  }

  if (linkUp()) {
    if (!apMode && !ntpStarted) {
      staEverConnected = true;
      Serial.printf("Wi-Fi connected, IP %s\n", WiFi.localIP().toString().c_str());
      configTime(0, 0, "pool.ntp.org", "time.google.com");
      ntpStarted = true;
    }
    if (!mqtt.connected()) {
      mqttConnect();
    } else {
      mqtt.loop();
      drainQueues();
      if (millis() - lastStatus > 10000) { publishStatus(); lastStatus = millis(); }
    }
  } else if (millis() - lastWifiMsg > 5000) {
    lastWifiMsg = millis();
    if (apMode)
      Serial.printf("AP '%s' (pass '%s', ESP32 at %s): waiting for the laptop to join... (%u poses buffered)\n",
                    AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str(), (unsigned)uxQueueMessagesWaiting(qPose));
    else
      Serial.printf("Wi-Fi: connecting to '%s'... (%u poses buffered)\n", WIFI_SSID,
                    (unsigned)uxQueueMessagesWaiting(qPose));
  }
  g_linkUp = mqtt.connected();
  delay(5);
}
