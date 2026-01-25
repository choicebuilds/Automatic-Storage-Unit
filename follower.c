/* =========================
   FOLLOWER (BLE Scanner + RSSI Homing) - ESP32 Arduino
   Behavior:
     - FORWARD: move ahead while RSSI trend improves
     - SEEK: stop, rotate/sweep, find yaw with strongest RSSI, face it, then FORWARD

   Install (Library Manager):
     - NimBLE-Arduino
     - (Optional) MPU6050 / I2Cdevlib if you want real yaw, otherwise use encoder/gyro

   IMPORTANT:
     - You MUST replace motor control stubs with your driver pins.
     - IMU yaw here is a placeholder. You need a gyro-based yaw integration to turn to angles.

========================= */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>

// Match leader UUID
static const char* kServiceUUID = "7b0c6db2-2a7b-4b93-9d2d-6c8d6b3d4c10";

// ---------- Tuning ----------
static const float kAlpha = 0.12f;         // RSSI EMA smoothing
static const int   kBadTrendCount = 12;    // ~0.6 s at 20 Hz
static const float kTrendThreshDb = 0.6f;  // "getting worse" threshold
static const int   kScanHz = 20;           // control loop rate
static const int   kSeekSweepDeg = 200;    // sweep range (180-360 typical)
static const int   kSeekStepDeg = 8;       // step size in sweep
static const int   kForwardPwm = 160;      // motor PWM
static const int   kTurnPwm = 140;         // turning PWM
static const int   kStopPwm = 0;

// ---------- BLE shared state ----------
volatile int   g_lastRSSI = -127;
volatile bool  g_seen = false;
volatile uint32_t g_lastSeenMs = 0;

// ---------- RSSI filtered ----------
float rssiFilt = -127.0f;
float rssiPrev = -127.0f;
int badTrend = 0;

// ---------- State machine ----------
enum State { FORWARD, SEEK };
State state = SEEK;

// ---------- IMU / yaw (placeholder) ----------
float yawDeg = 0.0f;  // You must implement updateYaw() and turning-to-angle based on your IMU.

// ================= Motor control (REPLACE THESE) =================
// Example: differential drive using two motors with direction + PWM
// Fill with your pin mapping (H-bridge / driver).
void motorsStop() {
  // TODO: set both motors to 0
}

void motorsForward(int pwm) {
  // TODO: set both motors forward at pwm
}

void motorsTurnLeft(int pwm) {
  // TODO: left motor reverse, right motor forward (or slow left, fast right)
}

void motorsTurnRight(int pwm) {
  // TODO: right motor reverse, left motor forward (or slow right, fast left)
}

// ================= IMU yaw (REPLACE THESE) =================
// You need: integrate gyro Z rate over time -> yawDeg.
// If you have encoders, you can estimate yaw from wheel odometry instead.
void imuInit() {
  Wire.begin();
  // TODO: init your IMU here (MPU6050, ICM20948, etc.)
}

void updateYaw() {
  // TODO: read gyro z, integrate dt -> yawDeg
  // yawDeg = wrap360(yawDeg);
}

float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// Turn to a target yaw (blocking). Replace with your real yaw feedback.
// If you do not have yaw, you can turn for fixed time per degree (less reliable).
void turnToYaw(float targetYawDeg) {
  targetYawDeg = wrap360(targetYawDeg);

  const uint32_t start = millis();
  const uint32_t timeoutMs = 6000;

  while (millis() - start < timeoutMs) {
    updateYaw();

    float cur = wrap360(yawDeg);
    float diff = targetYawDeg - cur;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    if (fabs(diff) < 6.0f) {
      motorsStop();
      return;
    }

    if (diff > 0) motorsTurnLeft(kTurnPwm);
    else motorsTurnRight(kTurnPwm);

    delay(10);
  }

  motorsStop();
}

// ================= BLE callbacks =================
class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (!dev->isAdvertisingService(NimBLEUUID(kServiceUUID))) return;

    g_lastRSSI = dev->getRSSI();
    g_seen = true;
    g_lastSeenMs = millis();
  }
};

NimBLEScan* scanner = nullptr;

void bleInit() {
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  scanner = NimBLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
  scanner->setActiveScan(true);     // more responsive RSSI, more power
  scanner->setInterval(45);
  scanner->setWindow(30);
  scanner->start(0, nullptr, false); // continuous scan
}

// ================= Seeking behavior =================
// Sweep left and right to find best RSSI and remember yaw of peak.
float seekFindBestYaw() {
  // Ensure we have a baseline RSSI
  uint32_t t0 = millis();
  while (!g_seen && millis() - t0 < 1500) delay(10);

  float bestR = -127.0f;
  float bestYaw = wrap360(yawDeg);

  // Sweep: turn in place step-by-step and sample RSSI
  float startYaw = wrap360(yawDeg);
  float targetEnd = wrap360(startYaw + kSeekSweepDeg);

  // Decide turn direction: always left for simplicity
  float curTarget = startYaw;

  for (int step = 0; step <= (kSeekSweepDeg / kSeekStepDeg); step++) {
    curTarget = wrap360(startYaw + step * kSeekStepDeg);
    turnToYaw(curTarget);

    // settle + gather a few RSSI samples
    float accum = 0.0f;
    int n = 0;
    uint32_t t = millis();
    while (millis() - t < 140) {
      if (g_seen && (millis() - g_lastSeenMs) < 500) {
        accum += (float)g_lastRSSI;
        n++;
      }
      delay(10);
    }

    float r = (n > 0) ? (accum / n) : -127.0f;
    if (r > bestR) {
      bestR = r;
      bestYaw = curTarget;
    }
  }

  return bestYaw;
}

void setup() {
  Serial.begin(115200);
  imuInit();
  bleInit();

  motorsStop();

  // Give time for first scan results
  delay(500);
  state = SEEK;
}

void loop() {
  static uint32_t lastTick = 0;
  const uint32_t periodMs = 1000 / kScanHz;

  if (millis() - lastTick < periodMs) return;
  lastTick = millis();

  updateYaw();

  // If beacon lost, stop and seek
  bool lost = (!g_seen) || (millis() - g_lastSeenMs > 1200);
  if (lost) {
    motorsStop();
    state = SEEK;
  }

  // Update filtered RSSI
  int rssiNow = g_lastRSSI;
  rssiFilt = kAlpha * (float)rssiNow + (1.0f - kAlpha) * rssiFilt;

  float trend = rssiFilt - rssiPrev;
  rssiPrev = rssiFilt;

  switch (state) {
    case FORWARD: {
      // If signal worsening consistently, seek a better heading
      if (trend < -kTrendThreshDb) badTrend++;
      else badTrend = max(0, badTrend - 1);

      motorsForward(kForwardPwm);

      if (badTrend >= kBadTrendCount) {
        motorsStop();
        badTrend = 0;
        state = SEEK;
      }
    } break;

    case SEEK: {
      motorsStop();

      float bestYaw = seekFindBestYaw();
      turnToYaw(bestYaw);

      // After aligning, go forward
      badTrend = 0;
      state = FORWARD;
    } break;
  }

  // Debug
  static uint32_t dbg = 0;
  if (millis() - dbg > 500) {
    dbg = millis();
    Serial.print("state=");
    Serial.print(state == FORWARD ? "FORWARD" : "SEEK");
    Serial.print(" rssiNow=");
    Serial.print(rssiNow);
    Serial.print(" rssiFilt=");
    Serial.print(rssiFilt, 1);
    Serial.print(" yaw=");
    Serial.println(wrap360(yawDeg), 1);
  }
}
