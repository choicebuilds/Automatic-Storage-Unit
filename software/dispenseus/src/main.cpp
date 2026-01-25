#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>
#include <string>

// -------------------- USER PINS --------------------
static const int PIN_PWM_LEFT  = 5;    // D5
static const int PIN_PWM_RIGHT = 19;   // D19

static const int SERVO1_PIN = 13;      // change if needed
static const int SERVO2_PIN = 14;      // change if needed

static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// -------------------- MOTOR --------------------
static const int CH_LEFT  = 0;
static const int CH_RIGHT = 1;
static const int PWM_FREQ_HZ  = 20000;
static const int PWM_RES_BITS = 8;

static inline int clamp255(int v) { return (v < 0) ? 0 : (v > 255) ? 255 : v; }

void motorsInit() {
  ledcSetup(CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_PWM_LEFT,  CH_LEFT);
  ledcAttachPin(PIN_PWM_RIGHT, CH_RIGHT);
  ledcWrite(CH_LEFT, 0);
  ledcWrite(CH_RIGHT, 0);
}
void motorsStop() { ledcWrite(CH_LEFT, 0); ledcWrite(CH_RIGHT, 0); }
void motorsForward(int pwm) {
  pwm = clamp255(pwm);
  ledcWrite(CH_LEFT, pwm);
  ledcWrite(CH_RIGHT, pwm);
}

void motorsTurnLeft(int pwm) {
  pwm = clamp255(pwm);
  ledcWrite(CH_LEFT, pwm / 3);
  ledcWrite(CH_RIGHT, pwm);
}
void motorsTurnRight(int pwm) {
  pwm = clamp255(pwm);
  ledcWrite(CH_LEFT, pwm);
  ledcWrite(CH_RIGHT, pwm / 3);
}

// -------------------- IMU --------------------
static const uint8_t MPU_ADDR = 0x68;

static const uint8_t REG_WHO_AM_I    = 0x75;
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_PWR_MGMT_2  = 0x6C;
static const uint8_t REG_CONFIG      = 0x1A;
static const uint8_t REG_SMPLRT_DIV  = 0x19;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;

float gyroZBias_dps = 0.0f;
uint32_t lastImuMs = 0;
float yawDeg = 0.0f;

static inline float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}
static inline float angleDiffDeg(float target, float cur) {
  float d = wrap360(target) - wrap360(cur);
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}
bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)addr, (int)n, (int)true);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
int16_t readInt16BE(uint8_t addr, uint8_t regH) {
  uint8_t b[2];
  if (!i2cReadN(addr, regH, b, 2)) return 0;
  return (int16_t)((b[0] << 8) | b[1]);
}
float readGyroZ_dps() {
  int16_t gz = readInt16BE(MPU_ADDR, REG_GYRO_ZOUT_H);
  return ((float)gz) / 131.0f;
}

void imuInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  uint8_t who = 0;
  if (!i2cReadN(MPU_ADDR, REG_WHO_AM_I, &who, 1)) {
    Serial.println("MPU: I2C read failed. Check wiring/address.");
    while (true) delay(1000);
  }
  Serial.print("MPU WHO_AM_I=0x");
  Serial.println(who, HEX);

  i2cWrite8(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
  delay(50);
  i2cWrite8(MPU_ADDR, REG_PWR_MGMT_2, 0x00);

  i2cWrite8(MPU_ADDR, REG_CONFIG, 0x03);
  i2cWrite8(MPU_ADDR, REG_SMPLRT_DIV, 0x04);
  i2cWrite8(MPU_ADDR, REG_GYRO_CONFIG, 0x00);

  Serial.println("Calibrating gyro bias (~2s still)...");
  const int N = 200;
  float sum = 0.0f;
  for (int i = 0; i < N; i++) { sum += readGyroZ_dps(); delay(10); }
  gyroZBias_dps = sum / N;
  Serial.print("gyroZBias_dps=");
  Serial.println(gyroZBias_dps, 4);

  yawDeg = 0.0f;
  lastImuMs = millis();
}

void updateYaw() {
  uint32_t now = millis();
  if (lastImuMs == 0) { lastImuMs = now; return; }
  float dt = (now - lastImuMs) / 1000.0f;
  lastImuMs = now;

  float gz = readGyroZ_dps() - gyroZBias_dps;
  yawDeg = wrap360(yawDeg + gz * dt);
}

// -------------------- BLE --------------------
volatile int      g_lastRSSI = -127;
volatile bool     g_seen = false;
volatile uint32_t g_lastSeenMs = 0;

volatile bool     g_cmdPending = false;
volatile uint8_t  g_cmdBtn = 0;
volatile uint8_t  g_cmdSeq = 0;

static void handleAdv(NimBLEAdvertisedDevice* dev) {
  g_lastRSSI = dev->getRSSI();
  g_seen = true;
  g_lastSeenMs = millis();

  std::string m = dev->getManufacturerData();
  if (m.size() >= 3 && (uint8_t)m[0] == 0xAA) {
    uint8_t seq = (uint8_t)m[1];
    uint8_t btn = (uint8_t)m[2];

    static uint8_t lastSeq = 0;
    if (seq != lastSeq) {
      lastSeq = seq;
      g_cmdSeq = seq;
      g_cmdBtn = btn;
      g_cmdPending = true;

      Serial.print("[RX] btn=");
      Serial.print((int)btn);
      Serial.print(" seq=");
      Serial.print((int)seq);
      Serial.print(" rssi=");
      Serial.println(dev->getRSSI());
    }
  }
}

class ScanCB : public NimBLEScanCallbacks {
  void onDiscovered(NimBLEAdvertisedDevice* dev) { handleAdv(dev); }
  void onResult(NimBLEAdvertisedDevice* dev) { handleAdv(dev); }
};

NimBLEScan* scanner = nullptr;

void bleInit() {
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  scanner = NimBLEDevice::getScan();
  scanner->setScanCallbacks(new ScanCB(), true);
  scanner->setActiveScan(true);
  scanner->setInterval(80);
  scanner->setWindow(79);

  Serial.println("Starting BLE scan...");
  scanner->start(0, false);
  Serial.println("BLE scan started.");
}

// -------------------- homing --------------------
static const int   kControlHz        = 50;
static const float kRssiAlpha        = 0.12f;
static const uint32_t kLostMs        = 1200;

static const float kBadTrendThreshDb = 0.6f;
static const int   kBadTrendCount    = 18;

static const float kSeekSweepDeg     = 220.0f;
static const float kSeekStepDeg      = 10.0f;
static const uint32_t kSeekMaxMs     = 9000;

static const float kYawDeadbandDeg   = 6.0f;
static const uint32_t kTurnMaxMs     = 2500;
static const int   kTurnPwm          = 140;

static const int   kForwardPwm       = 160;

float rssiFilt = -127.0f;
float rssiPrev = -127.0f;
int badTrend = 0;

enum Mode { MODE_FORWARD, MODE_SEEK_SWEEP, MODE_TURN_TO_BEST };
Mode mode = MODE_SEEK_SWEEP;

float turnTargetYaw = 0.0f;
uint32_t turnStartMs = 0;

float seekStartYaw = 0.0f;
float seekNextSampleYaw = 0.0f;
uint32_t seekStartMs = 0;

float seekBestYaw = 0.0f;
float seekBestRssi = -127.0f;

void startTurnTo(float targetYaw) {
  turnTargetYaw = wrap360(targetYaw);
  turnStartMs = millis();
  mode = MODE_TURN_TO_BEST;
}
void runTurnController() {
  float diff = angleDiffDeg(turnTargetYaw, yawDeg);

  if (millis() - turnStartMs > kTurnMaxMs) {
    motorsStop();
    mode = MODE_FORWARD;
    return;
  }
  if (fabs(diff) <= kYawDeadbandDeg) {
    motorsStop();
    mode = MODE_FORWARD;
    return;
  }
  if (diff > 0) motorsTurnLeft(kTurnPwm);
  else motorsTurnRight(kTurnPwm);
}

void startSeekSweep() {
  seekStartMs = millis();
  seekStartYaw = yawDeg;
  seekNextSampleYaw = seekStartYaw;
  seekBestYaw = yawDeg;
  seekBestRssi = -127.0f;
  mode = MODE_SEEK_SWEEP;
}

void runSeekSweep() {
  if (millis() - seekStartMs > kSeekMaxMs) {
    motorsStop();
    startTurnTo(seekBestYaw);
    return;
  }

  bool lost = (!g_seen) || (millis() - g_lastSeenMs > kLostMs);
  if (lost) {
    motorsTurnLeft(kTurnPwm);
    return;
  }

  motorsTurnLeft(kTurnPwm);

  float progressed = wrap360(yawDeg) - wrap360(seekStartYaw);
  if (progressed < 0) progressed += 360.0f;

  float nextProg = wrap360(seekNextSampleYaw) - wrap360(seekStartYaw);
  if (nextProg < 0) nextProg += 360.0f;

  if (progressed >= nextProg) {
    float r = rssiFilt;
    if (r > seekBestRssi) { seekBestRssi = r; seekBestYaw = yawDeg; }
    seekNextSampleYaw = wrap360(seekNextSampleYaw + kSeekStepDeg);
  }

  if (progressed >= kSeekSweepDeg) {
    motorsStop();
    startTurnTo(seekBestYaw);
  }
}

// -------------------- Servo --------------------
static const float STEP_DEG = 22.5f;

static const uint32_t SERVO1_HOLD_MS = 2000;
static const uint32_t SERVO2_OPEN_MS = 400;
static const uint32_t SERVO2_HOLD_MS = 400;
static const uint32_t SERVO2_CLOSE_MS= 400;

static const float SERVO2_CLOSED_DEG = 0.0f;
static const float SERVO2_OPEN_DEG   = 90.0f;

Servo servo1, servo2;

enum ServoMode { S_IDLE, S_MOVE1_SETTLE, S_HOLD1, S_S2_OPEN, S_S2_HOLD, S_S2_CLOSE };
ServoMode sMode = S_IDLE;
uint32_t sT0 = 0;

float servo1Pos = 0.0f;

static inline int nearestStepIdx(float deg) {
  int idx = (int)lround(deg / STEP_DEG);
  if (idx < 0) idx = 0;
  if (idx > 8) idx = 8; // 0..180 by 22.5
  return idx;
}
static inline float idxToDeg(int idx) {
  float d = idx * STEP_DEG;
  if (d < 0) d = 0;
  if (d > 180) d = 180;
  return d;
}

void servoStartSequenceTo(float targetDeg) {
  if (targetDeg < 0) targetDeg = 0;
  if (targetDeg > 180) targetDeg = 180;

  servo1.write((int)lround(targetDeg));
  servo1Pos = targetDeg;

  sMode = S_MOVE1_SETTLE;
  sT0 = millis();
}

void servoTick() {
  uint32_t now = millis();
  switch (sMode) {
    case S_IDLE: return;

    case S_MOVE1_SETTLE:
      if (now - sT0 >= 250) { sMode = S_HOLD1; sT0 = now; }
      break;

    case S_HOLD1:
      if (now - sT0 >= SERVO1_HOLD_MS) {
        servo2.write((int)lround(SERVO2_OPEN_DEG));
        sMode = S_S2_OPEN; sT0 = now;
      }
      break;

    case S_S2_OPEN:
      if (now - sT0 >= SERVO2_OPEN_MS) { sMode = S_S2_HOLD; sT0 = now; }
      break;

    case S_S2_HOLD:
      if (now - sT0 >= SERVO2_HOLD_MS) {
        servo2.write((int)lround(SERVO2_CLOSED_DEG));
        sMode = S_S2_CLOSE; sT0 = now;
      }
      break;

    case S_S2_CLOSE:
      if (now - sT0 >= SERVO2_CLOSE_MS) sMode = S_IDLE;
      break;
  }
}

void handleButton(uint8_t btn) {
  if (sMode != S_IDLE) return;

  if (btn == 1) {
    int idx = nearestStepIdx(servo1Pos);
    idx = min(idx + 1, 8);
    servoStartSequenceTo(idxToDeg(idx));
  } else if (btn == 2) {
    servoStartSequenceTo(0.0f);
  } else if (btn == 3) {
    servoStartSequenceTo(180.0f);
  }
}

// -------------------- setup/loop --------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("FOLLOWER BOOT");

  motorsInit();
  motorsStop();

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);
  servo1Pos = 0.0f;
  servo1.write(0);
  servo2.write((int)lround(SERVO2_CLOSED_DEG));

  imuInit();
  bleInit();

  startSeekSweep();

  Serial.println("Follower ready.");
}

void loop() {
  static uint32_t lastTick = 0;
  const uint32_t periodMs = 1000 / kControlHz;
  if (millis() - lastTick < periodMs) return;
  lastTick = millis();

  updateYaw();

  // consume command
  if (g_cmdPending) {
    g_cmdPending = false;
    Serial.print("CMD consumed btn=");
    Serial.print((int)g_cmdBtn);
    Serial.print(" seq=");
    Serial.println((int)g_cmdSeq);
    handleButton(g_cmdBtn);
  }

  servoTick();

  // RSSI filter update
  int rssiNow = g_lastRSSI;
  rssiFilt = kRssiAlpha * (float)rssiNow + (1.0f - kRssiAlpha) * rssiFilt;
  float trend = rssiFilt - rssiPrev;
  rssiPrev = rssiFilt;

  bool lost = (!g_seen) || (millis() - g_lastSeenMs > kLostMs);

  if (mode == MODE_FORWARD) {
    if (lost) {
      motorsStop();
      startSeekSweep();
    } else {
      motorsForward(kForwardPwm);

      if (trend < -kBadTrendThreshDb) badTrend++;
      else badTrend = max(0, badTrend - 1);

      if (badTrend >= kBadTrendCount) {
        badTrend = 0;
        motorsStop();
        startSeekSweep();
      }
    }
  } else if (mode == MODE_SEEK_SWEEP) {
    runSeekSweep();
  } else {
    runTurnController();
  }

  // Telemetry (always prints)
  static uint32_t dbg = 0;
  if (millis() - dbg > 250) {
    dbg = millis();
    Serial.print("mode=");
    Serial.print(mode == MODE_FORWARD ? "FWD" : (mode == MODE_SEEK_SWEEP ? "SEEK" : "TURN"));
    Serial.print(" raw=");
    Serial.print(rssiNow);
    Serial.print(" filt=");
    Serial.print(rssiFilt, 1);
    Serial.print(" yaw=");
    Serial.print(wrap360(yawDeg), 1);
    Serial.print(" sMode=");
    Serial.println((int)sMode);
  }
}
