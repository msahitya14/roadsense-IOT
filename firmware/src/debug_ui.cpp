// On-car debugging with the course kit: SPI SSD1306 OLED pages, 3 LEDs, 2 buttons, buzzer.
// Pages (BTN1 cycles): DRIVE, IMU, CAL, NET.  BTN2 tap = reset pose, hold 2 s = gyro calib.
#include "debug_ui.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "config.h"
#include "secrets.h"
#ifdef AP_PASS
#define AP_PASS_UI AP_PASS
#else
#define AP_PASS_UI "roadsense24"
#endif

static Adafruit_SSD1306 oled(128, 64, &SPI, IoTPins::OLED_DC, IoTPins::OLED_RST, IoTPins::OLED_CS);
static bool oledOk = false;
static UiHooks hooks;
static portMUX_TYPE bannerMux = portMUX_INITIALIZER_UNLOCKED;
static char banner[22] = "";
static uint32_t bannerUntil = 0;
static const char* PAGE_NAMES[] = {"DRIVE", "IMU", "CAL", "NET"};
static constexpr int NUM_PAGES = 4;

bool uiBegin() {
#if USE_LEDS
  for (uint8_t p : {IoTPins::LED_1, IoTPins::LED_2, IoTPins::LED_3}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
#endif
#if USE_BUTTONS
  pinMode(IoTPins::BUTTON_1, INPUT_PULLUP);
  pinMode(IoTPins::BUTTON_2, INPUT_PULLUP);
#endif
#if USE_OLED
  SPI.begin(IoTPins::SPI_SCK, IoTPins::SPI_MISO, IoTPins::SPI_MOSI);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC);
  if (!oledOk) Serial.println("OLED init failed: check SPI wiring (CS 5, DC 16, RST 17, SCK 18, MOSI 23)");
  else {
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextWrap(false);
  }
#endif
  return oledOk;
}

void uiLines(const char* l1, const char* l2, const char* l3, const char* l4) {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(l1); oled.println(l2); oled.println(l3); oled.println(l4);
  oled.display();
}

void uiNotify(const String& msg, uint16_t ms) {
  portENTER_CRITICAL(&bannerMux);
  strlcpy(banner, msg.c_str(), sizeof(banner));
  bannerUntil = millis() + ms;
  portEXIT_CRITICAL(&bannerMux);
}

// ------------------------------------------------------------------ drawing helpers
static void header(const char* page, const NetState& n, int idx) {
  oled.fillRect(0, 0, 128, 9, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setCursor(1, 1);
  oled.printf("%s %s %c%c M%c", DEVICE_ID, page, n.ap ? 'A' : 'W', n.wifi ? '+' : '-', n.mqtt ? '+' : '-');
  oled.setCursor(110, 1);
  oled.printf("%d/%d", idx + 1, NUM_PAGES);
  oled.setTextColor(SSD1306_WHITE);
}

static void line(int row, const char* fmt, ...) {  // rows 1..6 below the header
  char b[32];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  oled.setCursor(0, 2 + row * 9);
  oled.print(b);
}

static void footer(const DebugSnap& d) {
  char b[22];
  portENTER_CRITICAL(&bannerMux);
  bool show = millis() < bannerUntil;
  strlcpy(b, banner, sizeof(b));
  portEXIT_CRITICAL(&bannerMux);
  if (show) {
    oled.fillRect(0, 55, 128, 9, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(1, 56);
    oled.print(b);
    oled.setTextColor(SSD1306_WHITE);
  } else {
    oled.setCursor(0, 56);
    oled.printf("%s pot%lu %s", d.moving ? "DRIVE" : "park ", (unsigned long)d.potholes, d.label);
  }
}

static void compass(int cx, int cy, int r, float hdgDeg) {
  oled.drawCircle(cx, cy, r, SSD1306_WHITE);
  // 0 deg = start direction, drawn pointing up; CCW positive -> arrow rotates left
  float a = hdgDeg * DEG_TO_RAD;
  int tx = cx - (int)(sinf(a) * (r - 2)), ty = cy - (int)(cosf(a) * (r - 2));
  oled.drawLine(cx, cy, tx, ty, SSD1306_WHITE);
  oled.fillCircle(tx, ty, 2, SSD1306_WHITE);
}

static void bar(int x, int y, int w, float v, float th, float full) {  // value bar with threshold tick
  oled.drawRect(x, y, w, 5, SSD1306_WHITE);
  int f = (int)constrain(v / full * (w - 2), 0.0f, (float)(w - 2));
  oled.fillRect(x + 1, y + 1, f, 3, SSD1306_WHITE);
  int t = x + 1 + (int)constrain(th / full * (w - 2), 0.0f, (float)(w - 2));
  oled.drawFastVLine(t, y - 2, 9, SSD1306_WHITE);
}

// ------------------------------------------------------------------ pages
static void pageDrive(const DebugSnap& d) {
  line(1, "x %7.2f m", d.x);
  line(2, "y %7.2f m", d.y);
  line(3, "hdg %5.1f", d.hdg);
  line(4, "d %6.1fm %.2fm/s", d.dist, d.spd);
  line(5, "%-6s vib %.3f", SURF_NAMES[d.surf], d.vib);
  compass(114, 24, 12, d.hdg);
}

static void pageImu(const DebugSnap& d) {
  line(1, "a %5.2f %5.2f %5.2f", d.ax, d.ay, d.az);
  line(2, "g %5.1f %5.1f %5.1f", d.gx, d.gy, d.gz);
  line(3, "b %5.2f %5.2f %5.2f", d.bx, d.by, d.bz);
  line(4, "yaw %6.1f dps %4.1fC", d.yawRate, d.tempC);
  line(5, "sd%.3f", d.accSd);
  bar(48, 48, 80, d.accSd, d.movTh, d.movTh * 4);  // tick = movTh: bar past tick -> DRIVE
}

static void pageCal(const DebugSnap& d) {
  line(1, "v0 %.3f m/s", d.v0);
  line(2, "gyroScale %.4f", d.gyroScale);
  line(3, "movTh %.3f g", d.movTh);
  line(4, "dcal %s %.1fs", d.dcalActive ? "RUN " : "idle", d.dcalTime);
  line(5, "tcal %s %.1fdeg", d.tcalActive ? "RUN " : "idle", d.tcalRaw);
}

static void pageNet(const NetState& n, const DebugSnap& d) {
  if (n.ap) {
    line(1, "AP %.18s", n.ssid);
    line(2, "pw %.18s", AP_PASS_UI);
    line(4, "cli %d rc%lu e%ld", n.apClients, (unsigned long)n.reconnects, (long)n.mqttErr);
  } else {
    line(1, "%.21s", n.ssid);
    line(2, "ip %s", n.wifi ? n.ip : "-");
    line(4, "rssi %d rc%lu e%ld", n.rssi, (unsigned long)n.reconnects, (long)n.mqttErr);
  }
  line(3, "mq %.15s %s", n.broker, n.mqtt ? "OK" : "--");
  line(5, "q%lu drop%lu imuE%lu", (unsigned long)n.queued, (unsigned long)n.dropped, (unsigned long)n.imuErr);
}

// ------------------------------------------------------------------ task
static void uiTask(void*) {
  int page = 0;
  bool b1Prev = false, b2Prev = false, b2Fired = false;
  uint32_t b2Down = 0, lastDraw = 0, redUntil = 0, lastPot = 0;
  for (;;) {
    uint32_t now = millis();
    DebugSnap d;
    hooks.est->snapshot(d);
    NetState n;
    hooks.getNet(n);

#if USE_BUTTONS
    bool b1 = digitalRead(IoTPins::BUTTON_1) == LOW, b2 = digitalRead(IoTPins::BUTTON_2) == LOW;
    if (b1 && !b1Prev) { page = (page + 1) % NUM_PAGES; lastDraw = 0; }
    if (b2 && !b2Prev) { b2Down = now; b2Fired = false; }
    if (b2 && !b2Fired && now - b2Down > 2000) {
      b2Fired = true;
      uiNotify("CALIB: keep still", 3000);
      hooks.sendCmd("calib");
    }
    if (!b2 && b2Prev && !b2Fired && now - b2Down > 30) { hooks.sendCmd("reset"); uiNotify("pose reset"); }
    b1Prev = b1; b2Prev = b2;
#endif

    if (d.potholes > lastPot) {  // new pothole: red LED + beep + banner
      redUntil = now + 400;
      uiNotify("** POTHOLE **", 1500);
#if USE_BUZZER
      tone(IoTPins::BUZZER, 2200, 60);
#endif
    }
    lastPot = d.potholes;

#if USE_LEDS
    digitalWrite(IoTPins::LED_3, d.moving);                                    // green: driving
    digitalWrite(IoTPins::LED_2, !n.mqtt || (n.queued > 1 && (now / 250) % 2));  // yellow: offline / buffering
    digitalWrite(IoTPins::LED_1, now < redUntil || (!d.calibrated && (now / 200) % 2));  // red: pothole / calibrating
#endif

    if (oledOk && now - lastDraw >= 1000 / OLED_FPS) {
      lastDraw = now;
      oled.clearDisplay();
      oled.setTextSize(1);
      if (!d.calibrated) {
        header("BOOT", n, page);
        line(2, "Calibrating gyro...");
        line(3, "KEEP THE CAR STILL");
        footer(d);
      } else {
        header(PAGE_NAMES[page], n, page);
        switch (page) {
          case 0: pageDrive(d); break;
          case 1: pageImu(d); break;
          case 2: pageCal(d); break;
          default: pageNet(n, d); break;
        }
        footer(d);
      }
      oled.display();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void uiStart(const UiHooks& h) {
  hooks = h;
  xTaskCreatePinnedToCore(uiTask, "ui", 4096, nullptr, 2, nullptr, 1);
}
