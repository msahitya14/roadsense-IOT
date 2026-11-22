#pragma once
#include <Arduino.h>
#include "imu.h"

// Runtime parameters (saved in NVS, changed with `set <key> <value>`).
struct Params {
  float v0;         // m/s   cruise speed while driving        (calibrated by `dcal`)
  float vibRef;     // g     vertical vibration RMS during dcal (used by spdMode 1)
  int   spdMode;    //       0: v = v0 while moving, 1: v = v0 * vib / vibRef, 2: v = v0 + kVib * vib
  float gyroScale;  //       yaw-rate multiplier                (calibrated by `tcal`)
  float movTh;      // g     |accel| std (0.25 s window) above which the car is "driving"
  float potNeg;     // g     vertical dip that arms the pothole detector (negative)
  float potPos;     // g     vertical spike within 250 ms of the dip -> pothole
  float potBig;     // g     any |vertical| spike above this -> pothole
  float surfT1;     // g RMS below -> "smooth"
  float surfT2;     // g RMS below -> "medium", above -> "rough"
  float kVib;       // m/s per g  (spdMode 2: v = v0 + kVib * vib)
};

struct PoseMsg {
  uint32_t ts, seq;
  float x, y, hdg, spd, dist, vib, accSd, conf;
  float drvS, yawRaw, vibInt;  // raw, parameter-free integrals since reset (for offline calibration fits)
  uint8_t surf, moving;
  char label[12];
};

struct PotholeMsg {
  uint32_t ts, seq;
  float x, y, zneg, zpos, pitch;
};

// Live values for the OLED (refreshed 10x per second, read from another task).
struct DebugSnap {
  bool calibrated, moving, dcalActive, tcalActive;
  uint8_t surf;
  float x, y, hdg, spd, dist;
  float accSd, vib, yawRate, azPeak;     // azPeak: max |vertical accel| in the last second
  float ax, ay, az, gx, gy, gz, tempC;   // latest raw sample
  float bx, by, bz;                      // gyro bias
  float v0, gyroScale, movTh, dcalTime, tcalRaw;
  uint32_t potholes;
  char label[12];
};

extern const char* SURF_NAMES[];  // 0 idle, 1 smooth, 2 medium, 3 rough

// Dead reckoning with only an MPU-6050:
//  * "driving" is detected from motor/road vibration (accel magnitude std),
//  * distance = calibrated speed x driving time (no wheel sensor),
//  * heading  = gyro yaw rate about the measured gravity axis, bias re-estimated whenever still.
class Estimator {
 public:
  void begin();
  bool calibrateStill(uint32_t ms, String& msg);  // car must not move
  void update(const ImuSample& s, float dt);
  bool poseReady(PoseMsg& out);
  bool potholeReady(PotholeMsg& out);
  String command(const String& line);
  String paramsJson() const;
  void snapshot(DebugSnap& out);  // thread-safe copy for the display task
  int streamHz = 0;  // raw serial stream rate (0 = off)

 private:
  void fillSnap(const ImuSample& s, float yaw);
  portMUX_TYPE snapMux_ = portMUX_INITIALIZER_UNLOCKED;
  DebugSnap snap_{};
  int snapCnt_ = 0;
  float azPeak_ = 0, azPeakPrev_ = 0;
  int azPeakN_ = 0;
  uint32_t potCount_ = 0;
  bool calibrated_ = false;

  void setDefaults();
  void save();
  uint8_t classify(float rms, float& conf) const;
  void flagPothole();

  Params p_;
  char label_[12] = "none";
  uint32_t seq_ = 0;

  // calibration state
  float bx_ = 0, by_ = 0, bz_ = 0;           // gyro bias, dps
  float gax_ = 0, gay_ = 0, gaz_ = 1;        // low-passed gravity vector, g
  float ux_ = 0, uy_ = 0, uz_ = 1, gMag_ = 1;  // unit "up" vector and |g|

  // pose
  float x_ = 0, y_ = 0, theta_ = 0, spd_ = 0, dist_ = 0;
  float drvS_ = 0, yawRawCum_ = 0, vibInt_ = 0;  // since reset: driving s, unscaled yaw deg, integral of vib dt
  bool moving_ = false;
  float stillTime_ = 0;

  // windows
  int mwN_ = 0; float mwSum_ = 0, mwSq_ = 0; int quietWins_ = 0; float accSd_ = 0;
  int swN_ = 0; float swSum_ = 0, swSq_ = 0; float vibRms_ = 0;
  int poseCnt_ = 0;

  // pothole detector
  bool armed_ = false; float armT_ = 0, refractory_ = 0;
  float zmin_ = 0, zmax_ = 0, pmax_ = 0;
  bool havePose_ = false, havePot_ = false;
  PoseMsg pose_{};
  PotholeMsg pot_{};

  // calibration runs
  bool dcalActive_ = false; float dcalTime_ = 0, dcalVib_ = 0, dcalDist0_ = 0;
  bool tcalActive_ = false; float tcalRaw_ = 0;
};
