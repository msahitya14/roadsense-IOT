// BLE transport. Binary frames keep the radio time small so poses can be sent at up to 100 Hz.
//
// FRAME FORMAT (little-endian, one frame per notification; must match tools/ble_bridge.py)
//   POSE    (71 B) <B I I I 11f B B 12s>  type, seq, t_ms, ts, x, y, hdg, spd, dist, vib, acc_sd, conf,
//                                          drv_s, yaw_raw, vib_int, surf, mv, label
//   POTHOLE (33 B) <B I I I 5f>           type, seq, t_ms, ts, x, y, z_neg, z_pos, pitch
//   STATUS         <B> + JSON text
//   INFO           <B B> + text           type, more-chunks-follow flag, UTF-8 text
//   RAW            <B I B H> + n*<6h>     type, t_ms of first sample, n, sample period us, samples
#include "ble_link.h"
#include <NimBLEDevice.h>

extern "C" int os_msys_num_free(void);  // NimBLE mbuf pool: free buffers for outgoing notifications

static const char* SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* RX_UUID  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static const char* TX_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

static NimBLEServer* srv = nullptr;
static NimBLECharacteristic* tx = nullptr;
static void (*cmdCb)(const char*) = nullptr;
static volatile bool subscribed = false;
static volatile uint16_t peerMtu = 23;
static volatile uint32_t nConnects = 0, nErrors = 0, nSent = 0;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* d) override {
    nConnects++;
    // ask for a 15-30 ms connection interval (macOS minimum is 15 ms): more packets per second
    s->updateConnParams(d->conn_handle, 12, 24, 0, 400);
  }
  void onDisconnect(NimBLEServer* s) override {
    subscribed = false;
    peerMtu = 23;
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, ble_gap_conn_desc* d) override { peerMtu = mtu; }
};

class TxCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, ble_gap_conn_desc* d, uint16_t v) override { subscribed = v != 0; }
  void onStatus(NimBLECharacteristic* c, Status s, int code) override {
    if (s == SUCCESS_NOTIFY) nSent++;
    else nErrors++;
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    while (!v.empty() && (v.back() == '\n' || v.back() == '\r')) v.pop_back();
    if (!v.empty() && cmdCb) cmdCb(v.c_str());
  }
};

void bleBegin(const char* name, void (*onCmd)(const char*)) {
  cmdCb = onCmd;
  NimBLEDevice::init(name);
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // +9 dBm: maximum range
  srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  NimBLEService* svc = srv->createService(SVC_UUID);
  tx = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  tx->setCallbacks(new TxCB());
  NimBLECharacteristic* rx = svc->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);  // name goes in the scan response
  adv->start();
}

bool bleConnected() { return srv && srv->getConnectedCount() > 0 && subscribed; }
uint16_t blePayloadMax() { return peerMtu > 3 ? peerMtu - 3 : 20; }
bool bleCanSend() { return bleConnected() && os_msys_num_free() >= 4; }
uint32_t bleConnects() { return nConnects; }
uint32_t bleNotifyErrors() { return nErrors; }
uint32_t bleFramesSent() { return nSent; }

static bool sendFrame(const uint8_t* b, size_t n) {
  if (!bleCanSend() || n > blePayloadMax()) return false;
  tx->notify(b, n, true);
  return true;
}

// little-endian packing helpers (ESP32 is little-endian, memcpy keeps it alignment-safe)
template <typename T>
static void put(uint8_t*& p, T v) { memcpy(p, &v, sizeof(T)); p += sizeof(T); }

bool bleSendPose(const PoseMsg& m) {
  uint8_t b[71], *p = b;
  put<uint8_t>(p, FRAME_POSE);
  put(p, m.seq); put(p, m.tMs); put(p, m.ts);
  for (float f : {m.x, m.y, m.hdg, m.spd, m.dist, m.vib, m.accSd, m.conf, m.drvS, m.yawRaw, m.vibInt}) put(p, f);
  put<uint8_t>(p, m.surf); put<uint8_t>(p, m.moving);
  memset(p, 0, 12); memcpy(p, m.label, strnlen(m.label, 12)); p += 12;
  return sendFrame(b, p - b);
}

bool bleSendPothole(const PotholeMsg& m) {
  uint8_t b[33], *p = b;
  put<uint8_t>(p, FRAME_POTHOLE);
  put(p, m.seq); put(p, m.tMs); put(p, m.ts);
  for (float f : {m.x, m.y, m.zneg, m.zpos, m.pitch}) put(p, f);
  return sendFrame(b, p - b);
}

bool bleSendText(uint8_t type, const char* s) {
  size_t len = strlen(s), room = blePayloadMax() - (type == FRAME_INFO ? 2 : 1);
  if (type == FRAME_STATUS) {
    if (len > room) return false;
    uint8_t b[256];
    b[0] = type; memcpy(b + 1, s, len);
    return sendFrame(b, len + 1);
  }
  do {  // INFO: split long help texts into chunks
    size_t n = len > room ? room : len;
    uint8_t b[256];
    b[0] = type; b[1] = len > n;
    memcpy(b + 2, s, n);
    if (!sendFrame(b, n + 2)) return false;
    s += n; len -= n;
  } while (len);
  return true;
}

int bleRawPerFrame() {
  int n = ((int)blePayloadMax() - 8) / (int)sizeof(RawSample);
  return n < 1 ? 1 : (n > 20 ? 20 : n);
}

bool bleSendRaw(uint32_t tMsFirst, const RawSample* s, uint8_t n) {
  uint8_t b[8 + 20 * sizeof(RawSample)], *p = b;
  put<uint8_t>(p, FRAME_RAW); put(p, tMsFirst); put<uint8_t>(p, n); put<uint16_t>(p, 1000000 / 200);
  memcpy(p, s, n * sizeof(RawSample)); p += n * sizeof(RawSample);
  return sendFrame(b, p - b);
}
