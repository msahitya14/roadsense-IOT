#include "estimator.h"
#include <Preferences.h>
#include <math.h>
#include <time.h>
#include "config.h"

const char* SURF_NAMES[] = {"idle", "smooth", "medium", "rough"};
static Preferences prefs;

static uint32_t nowTs() {
  time_t t = time(nullptr);
  return t > 1700000000 ? (uint32_t)t : millis() / 1000;  // epoch if NTP synced, else uptime
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void Estimator::setDefaults() {
  p_ = {0.80f, 0.15f, 0, 1.0f, 0.035f, -0.60f, 0.90f, 1.80f, 0.08f, 0.20f, 0.0f};
}

void Estimator::begin() {
  setDefaults();
  prefs.begin("rs", false);
  p_.v0 = prefs.getFloat("v0", p_.v0);
  p_.vibRef = prefs.getFloat("vibRef", p_.vibRef);
  p_.spdMode = prefs.getInt("spdMode", p_.spdMode);
  p_.gyroScale = prefs.getFloat("gyroScale", p_.gyroScale);
  p_.movTh = prefs.getFloat("movTh", p_.movTh);
  p_.potNeg = prefs.getFloat("potNeg", p_.potNeg);
  p_.potPos = prefs.getFloat("potPos", p_.potPos);
  p_.potBig = prefs.getFloat("potBig", p_.potBig);
  p_.surfT1 = prefs.getFloat("surfT1", p_.surfT1);
  p_.surfT2 = prefs.getFloat("surfT2", p_.surfT2);
  p_.kVib = prefs.getFloat("kVib", p_.kVib);
}

void Estimator::save() {
  prefs.putFloat("v0", p_.v0);
  prefs.putFloat("vibRef", p_.vibRef);
  prefs.putInt("spdMode", p_.spdMode);
  prefs.putFloat("gyroScale", p_.gyroScale);
  prefs.putFloat("movTh", p_.movTh);
  prefs.putFloat("potNeg", p_.potNeg);
  prefs.putFloat("potPos", p_.potPos);
  prefs.putFloat("potBig", p_.potBig);
  prefs.putFloat("surfT1", p_.surfT1);
  prefs.putFloat("surfT2", p_.surfT2);
  prefs.putFloat("kVib", p_.kVib);
}

bool Estimator::calibrateStill(uint32_t ms, String& msg) {
  double sgx = 0, sgy = 0, sgz = 0, sax = 0, say = 0, saz = 0, sm = 0, sm2 = 0;
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    ImuSample s;
    if (imuRead(s)) {
      sgx += s.gx; sgy += s.gy; sgz += s.gz;
      sax += s.ax; say += s.ay; saz += s.az;
      float m = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
      sm += m; sm2 += m * m;
      n++;
    }
    delay(1000 / SAMPLE_HZ);
  }
  if (n < 50) { msg = "calib FAILED: IMU not responding"; return false; }
  double mean = sm / n, sd = sqrt(fmax(sm2 / n - mean * mean, 0.0));
  if (sd > 0.02) {
    msg = String("calib FAILED: car moved (accel sd ") + String(sd, 3) + " g). Keep it still.";
    return false;
  }
  bx_ = sgx / n; by_ = sgy / n; bz_ = sgz / n;
  gax_ = sax / n; gay_ = say / n; gaz_ = saz / n;
  gMag_ = sqrtf(gax_ * gax_ + gay_ * gay_ + gaz_ * gaz_);
  ux_ = gax_ / gMag_; uy_ = gay_ / gMag_; uz_ = gaz_ / gMag_;
  moving_ = false;
  stillTime_ = 10;
  calibrated_ = true;
  char b[200];
  snprintf(b, sizeof(b),
           "calib OK: gyro bias (%.2f, %.2f, %.2f) dps, |g| %.3f, up axis (%.2f, %.2f, %.2f), n=%d",
           bx_, by_, bz_, gMag_, ux_, uy_, uz_, n);
  msg = b;
  return true;
}

uint8_t Estimator::classify(float rms, float& conf) const {
  if (!moving_) { conf = 1.0f; return 0; }
  uint8_t c = rms < p_.surfT1 ? 1 : (rms < p_.surfT2 ? 2 : 3);
  // confidence: how far from the nearest class boundary (relative)
  float d1 = fabsf(rms - p_.surfT1) / p_.surfT1;
  float d2 = fabsf(rms - p_.surfT2) / p_.surfT2;
  conf = clampf(0.5f + fminf(d1, d2), 0.5f, 0.99f);
  return c;
}

void Estimator::flagPothole() {
  pot_.ts = nowTs();
  pot_.seq = ++seq_;
  pot_.x = x_; pot_.y = y_;
  pot_.zneg = zmin_; pot_.zpos = zmax_; pot_.pitch = pmax_;
  havePot_ = true;
  potCount_++;
  armed_ = false;
  refractory_ = 0.6f;
}

void Estimator::update(const ImuSample& s, float dt) {
  // bias-corrected gyro and projections onto the "up" axis
  float wx = s.gx - bx_, wy = s.gy - by_, wz = s.gz - bz_;
  float yawRaw = wx * ux_ + wy * uy_ + wz * uz_;         // dps, CCW seen from above
  float yaw = yawRaw * p_.gyroScale;
  float px = wx - yawRaw * ux_, py = wy - yawRaw * uy_, pz = wz - yawRaw * uz_;
  float pitchRate = sqrtf(px * px + py * py + pz * pz);  // pitch/roll rate magnitude
  float amag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  float azl = (s.ax * ux_ + s.ay * uy_ + s.az * uz_) - gMag_;  // vertical linear accel, g

  // ---- driving detection: |accel| std over 0.25 s, 0.75 s hang time
  mwN_++; mwSum_ += amag; mwSq_ += amag * amag;
  if (mwN_ >= SAMPLE_HZ / 4) {
    float m = mwSum_ / mwN_;
    accSd_ = sqrtf(fmaxf(mwSq_ / mwN_ - m * m, 0));
    if (accSd_ > p_.movTh) { moving_ = true; quietWins_ = 0; }
    else if (moving_ && ++quietWins_ >= 3) moving_ = false;
    mwN_ = 0; mwSum_ = mwSq_ = 0;
  }
  stillTime_ = moving_ ? 0 : stillTime_ + dt;

  // ---- while parked: track gyro bias and gravity direction (zero-velocity update)
  float wmag = sqrtf(wx * wx + wy * wy + wz * wz);
  if (!moving_ && stillTime_ > 2.0f && wmag < 3.0f) {
    const float a = 0.002f;  // ~2.5 s time constant
    bx_ += a * wx; by_ += a * wy; bz_ += a * wz;
    gax_ += a * (s.ax - gax_); gay_ += a * (s.ay - gay_); gaz_ += a * (s.az - gaz_);
    float g = sqrtf(gax_ * gax_ + gay_ * gay_ + gaz_ * gaz_);
    if (g > 0.5f) { gMag_ = g; ux_ = gax_ / g; uy_ = gay_ / g; uz_ = gaz_ / g; }
  }

  // ---- heading (deg, CCW positive, 0 = direction the car faced at reset)
  if (tcalActive_) tcalRaw_ += yawRaw * dt;
  bool deadband = !moving_ && stillTime_ > 1.0f && fabsf(yaw) < 2.0f;
  if (!deadband) { theta_ += yaw * dt; yawRawCum_ += yawRaw * dt; }
  while (theta_ >= 180) theta_ -= 360;
  while (theta_ < -180) theta_ += 360;

  // ---- speed and position
  float v = 0;
  if (moving_) {
    if (p_.spdMode == 1) v = p_.v0 * clampf(vibRms_ / p_.vibRef, 0.3f, 2.0f);
    else if (p_.spdMode == 2) v = fmaxf(p_.v0 + p_.kVib * vibRms_, 0.0f);
    else v = p_.v0;
    drvS_ += dt;
    vibInt_ += vibRms_ * dt;
  }
  spd_ = v;
  float th = theta_ * DEG_TO_RAD;
  x_ += v * cosf(th) * dt;
  y_ += v * sinf(th) * dt;
  dist_ += v * dt;
  if (dcalActive_ && moving_) { dcalTime_ += dt; dcalVib_ += vibRms_ * dt; }

  // ---- surface: vertical vibration RMS over 1 s
  swN_++; swSum_ += azl; swSq_ += azl * azl;
  if (swN_ >= SAMPLE_HZ) {
    float m = swSum_ / swN_;
    vibRms_ = sqrtf(fmaxf(swSq_ / swN_ - m * m, 0));
    swN_ = 0; swSum_ = swSq_ = 0;
  }

  // ---- pothole: dip (wheel drops) followed by a spike (wheel hits far edge), or one big spike
  if (refractory_ > 0) refractory_ -= dt;
  if (moving_ && refractory_ <= 0) {
    if (armed_) {
      armT_ += dt;
      zmin_ = fminf(zmin_, azl); zmax_ = fmaxf(zmax_, azl); pmax_ = fmaxf(pmax_, pitchRate);
      if (azl > p_.potPos) flagPothole();
      else if (armT_ > 0.25f) armed_ = false;
    } else if (azl < p_.potNeg) {
      armed_ = true; armT_ = 0; zmin_ = zmax_ = azl; pmax_ = pitchRate;
    } else if (fabsf(azl) > p_.potBig) {
      zmin_ = fminf(azl, 0); zmax_ = fmaxf(azl, 0); pmax_ = pitchRate;
      flagPothole();
    }
  }

  // ---- debug snapshot for the OLED (10 Hz), peak vertical accel over 1 s
  azPeak_ = fmaxf(azPeak_, fabsf(azl));
  if (++azPeakN_ >= SAMPLE_HZ) { azPeakPrev_ = azPeak_; azPeak_ = 0; azPeakN_ = 0; }
  if (++snapCnt_ >= SAMPLE_HZ / 10) { snapCnt_ = 0; fillSnap(s, yaw); }

  // ---- 1 Hz pose message
  if (++poseCnt_ >= SAMPLE_HZ * POSE_PERIOD_S) {
    poseCnt_ = 0;
    pose_.ts = nowTs();
    pose_.seq = ++seq_;
    pose_.x = x_; pose_.y = y_;
    pose_.hdg = theta_ < 0 ? theta_ + 360 : theta_;
    pose_.spd = spd_;
    pose_.dist = dist_;
    pose_.vib = vibRms_;
    pose_.accSd = accSd_;
    pose_.drvS = drvS_; pose_.yawRaw = yawRawCum_; pose_.vibInt = vibInt_;
    pose_.surf = classify(vibRms_, pose_.conf);
    pose_.moving = moving_;
    strlcpy(pose_.label, label_, sizeof(pose_.label));
    havePose_ = true;
  }
}

void Estimator::fillSnap(const ImuSample& s, float yaw) {
  DebugSnap d;
  d.calibrated = calibrated_; d.moving = moving_; d.dcalActive = dcalActive_; d.tcalActive = tcalActive_;
  float c;
  d.surf = classify(vibRms_, c);
  d.x = x_; d.y = y_; d.hdg = theta_ < 0 ? theta_ + 360 : theta_; d.spd = spd_; d.dist = dist_;
  d.accSd = accSd_; d.vib = vibRms_; d.yawRate = yaw; d.azPeak = fmaxf(azPeak_, azPeakPrev_);
  d.ax = s.ax; d.ay = s.ay; d.az = s.az; d.gx = s.gx; d.gy = s.gy; d.gz = s.gz; d.tempC = s.tempC;
  d.bx = bx_; d.by = by_; d.bz = bz_;
  d.v0 = p_.v0; d.gyroScale = p_.gyroScale; d.movTh = p_.movTh; d.dcalTime = dcalTime_; d.tcalRaw = tcalRaw_;
  d.potholes = potCount_;
  strlcpy(d.label, label_, sizeof(d.label));
  portENTER_CRITICAL(&snapMux_);
  snap_ = d;
  portEXIT_CRITICAL(&snapMux_);
}

void Estimator::snapshot(DebugSnap& out) {
  portENTER_CRITICAL(&snapMux_);
  out = snap_;
  portEXIT_CRITICAL(&snapMux_);
}

bool Estimator::poseReady(PoseMsg& out) {
  if (!havePose_) return false;
  out = pose_; havePose_ = false; return true;
}

bool Estimator::potholeReady(PotholeMsg& out) {
  if (!havePot_) return false;
  out = pot_; havePot_ = false; return true;
}

String Estimator::paramsJson() const {
  char b[360];
  snprintf(b, sizeof(b),
           "{\"v0\":%.4f,\"vibRef\":%.4f,\"spdMode\":%d,\"gyroScale\":%.4f,\"movTh\":%.4f,"
           "\"potNeg\":%.3f,\"potPos\":%.3f,\"potBig\":%.3f,\"surfT1\":%.4f,\"surfT2\":%.4f,\"kVib\":%.4f,"
           "\"bias\":[%.3f,%.3f,%.3f],\"label\":\"%s\"}",
           p_.v0, p_.vibRef, p_.spdMode, p_.gyroScale, p_.movTh, p_.potNeg, p_.potPos, p_.potBig,
           p_.surfT1, p_.surfT2, p_.kVib, bx_, by_, bz_, label_);
  return String(b);
}

static const char* HELP =
    "commands:\n"
    "  show                 print parameters\n"
    "  calib                re-measure gyro bias (car still, 2 s)\n"
    "  reset                pose and distance to 0 (car at start point, facing start direction)\n"
    "  anchor <x> <y> [hdg] re-anchor pose at a known waypoint (m, m, deg)\n"
    "  dcal start | dcal end <meters>   distance/speed calibration run\n"
    "  tcal start | tcal end <degrees>  gyro/turn calibration\n"
    "  set <key> <value>    keys: v0 vibRef spdMode kVib gyroScale movTh potNeg potPos potBig surfT1 surfT2\n"
    "  label <name>         tag following pose messages (asphalt/gravel/grass...) for data collection\n"
    "  stream <hz>          raw IMU CSV on serial (0 = off, max 100)\n"
    "  time <unix s>        set the clock (dashboard does this automatically)\n"
    "  defaults             restore default parameters";

String Estimator::command(const String& lineIn) {
  String line = lineIn;
  line.trim();
  char cmd[16] = {0}, arg[24] = {0};
  float v1 = NAN, v2 = NAN, v3 = NAN;
  sscanf(line.c_str(), "%15s", cmd);
  String c(cmd);
  c.toLowerCase();
  char b[300];

  if (c == "help" || c == "?") return HELP;
  if (c == "show" || c == "params") return "params " + paramsJson();

  if (c == "calib") {
    String msg;
    calibrateStill(2000, msg);
    return msg;
  }
  if (c == "reset") {
    x_ = y_ = theta_ = dist_ = 0;
    drvS_ = yawRawCum_ = vibInt_ = 0;
    potCount_ = 0;
    return "reset: pose (0,0) heading 0, distance 0";
  }
  if (c == "anchor") {
    int n = sscanf(line.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
    if (n < 2) return "usage: anchor <x> <y> [hdg]";
    x_ = v1; y_ = v2;
    if (n == 3) theta_ = v3 > 180 ? v3 - 360 : v3;
    snprintf(b, sizeof(b), "anchored at (%.2f, %.2f) hdg %.1f", x_, y_, theta_);
    return b;
  }
  if (c == "dcal") {
    int n = sscanf(line.c_str(), "%*s %23s %f", arg, &v1);
    String a(arg);
    if (a == "start") {
      dcalActive_ = true; dcalTime_ = dcalVib_ = 0; dcalDist0_ = dist_;
      return "dcal started: drive straight at your normal throttle, stop, then `dcal end <meters>`";
    }
    if (a == "end" && n == 2) {
      dcalActive_ = false;
      if (dcalTime_ < 1.0f) return "dcal FAILED: less than 1 s of driving detected (check movTh)";
      float est = dist_ - dcalDist0_;
      float oldV0 = p_.v0;
      p_.v0 = v1 / dcalTime_;
      p_.vibRef = fmaxf(dcalVib_ / dcalTime_, 0.005f);
      save();
      snprintf(b, sizeof(b),
               "dcal OK: %.2f m in %.2f s driving -> v0 %.3f m/s (was %.3f, estimate was %.2f m, err %.1f%%), vibRef %.3f g",
               v1, dcalTime_, p_.v0, oldV0, est, v1 > 0 ? 100.0f * (est - v1) / v1 : 0, p_.vibRef);
      return b;
    }
    snprintf(b, sizeof(b), "dcal %s: %.2f s driving so far. usage: dcal start | dcal end <meters>",
             dcalActive_ ? "running" : "idle", dcalTime_);
    return b;
  }
  if (c == "tcal") {
    int n = sscanf(line.c_str(), "%*s %23s %f", arg, &v1);
    String a(arg);
    if (a == "start") {
      tcalActive_ = true; tcalRaw_ = 0;
      return "tcal started: turn the car a known angle (e.g. 360 or 4x90), stop, then `tcal end <degrees>`";
    }
    if (a == "end" && n == 2) {
      tcalActive_ = false;
      if (fabsf(tcalRaw_) < 30) return "tcal FAILED: gyro saw < 30 deg of turning";
      float old = p_.gyroScale;
      p_.gyroScale = fabsf(v1) / fabsf(tcalRaw_);
      save();
      snprintf(b, sizeof(b), "tcal OK: gyro measured %.1f deg for true %.1f -> gyroScale %.4f (was %.4f)",
               tcalRaw_, v1, p_.gyroScale, old);
      return b;
    }
    snprintf(b, sizeof(b), "tcal %s: raw %.1f deg so far. usage: tcal start | tcal end <degrees>",
             tcalActive_ ? "running" : "idle", tcalRaw_);
    return b;
  }
  if (c == "set") {
    if (sscanf(line.c_str(), "%*s %23s %f", arg, &v1) != 2) return "usage: set <key> <value>";
    String k(arg);
    if (k == "v0") p_.v0 = v1;
    else if (k == "vibRef") p_.vibRef = v1;
    else if (k == "spdMode") p_.spdMode = (int)v1;
    else if (k == "gyroScale") p_.gyroScale = v1;
    else if (k == "movTh") p_.movTh = v1;
    else if (k == "potNeg") p_.potNeg = v1;
    else if (k == "potPos") p_.potPos = v1;
    else if (k == "potBig") p_.potBig = v1;
    else if (k == "surfT1") p_.surfT1 = v1;
    else if (k == "surfT2") p_.surfT2 = v1;
    else if (k == "kVib") p_.kVib = v1;
    else return "unknown key " + k;
    save();
    return "set " + k + " = " + String(v1, 4) + " (saved)";
  }
  if (c == "label") {
    if (sscanf(line.c_str(), "%*s %11s", label_) != 1) strlcpy(label_, "none", sizeof(label_));
    return String("label = ") + label_;
  }
  if (c == "stream") {
    int hz = 0;
    sscanf(line.c_str(), "%*s %d", &hz);
    streamHz = constrain(hz, 0, 100);
    return "stream " + String(streamHz) + " Hz (format: R,ms,ax,ay,az,gx,gy,gz,azl)";
  }
  if (c == "time") {  // clock sync from the dashboard (no NTP in AP mode)
    unsigned long t = 0;
    sscanf(line.c_str(), "%*s %lu", &t);
    if (t < 1700000000UL) return "usage: time <unix seconds>";
    struct timeval tv = {(time_t)t, 0};
    settimeofday(&tv, nullptr);
    return "clock set to " + String(t);
  }
  if (c == "defaults") {
    setDefaults(); save();
    return "defaults restored: " + paramsJson();
  }
  return "unknown command '" + line + "' (type help)";
}
