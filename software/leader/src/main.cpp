#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>

static const char* kName = "LEADER_CMD";

static const int BTN1 = 32;
static const int BTN2 = 33;
static const int BTN3 = 25;

static const uint32_t DEBOUNCE_MS = 35;

NimBLEAdvertising* adv = nullptr;

static bool last1 = true, last2 = true, last3 = true;
static uint32_t lastEdge1 = 0, lastEdge2 = 0, lastEdge3 = 0;

static uint8_t seq = 0;

static void applyAdvPayload(uint8_t btn) {
  uint8_t mfg[3] = {0xAA, seq, btn};

  NimBLEAdvertisementData ad;
  ad.setManufacturerData(std::string((char*)mfg, sizeof(mfg)));

  NimBLEAdvertisementData sr;
  sr.setName(kName);

  adv->stop();
  adv->clearData();
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  adv->start();

  Serial.print("ADV btn=");
  Serial.print((int)btn);
  Serial.print(" seq=");
  Serial.println((int)seq);
}

static void sendBtn(uint8_t btn) {
  seq++;
  applyAdvPayload(btn);
}

static void pollBtn(int pin, bool& lastState, uint32_t& lastEdgeMs, uint8_t btnNum) {
  bool nowState = digitalRead(pin); 
  if (nowState != lastState) {
    uint32_t now = millis();
    if (now - lastEdgeMs > DEBOUNCE_MS) {
      lastEdgeMs = now;
      lastState = nowState;
      if (nowState == LOW) sendBtn(btnNum);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);

  NimBLEDevice::init(kName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  adv = NimBLEDevice::getAdvertising();
  adv->setMinInterval(80);
  adv->setMaxInterval(120);

  applyAdvPayload(0);

  Serial.println("Leader ready. Press buttons 1/2/3.");
}

void loop() {
  pollBtn(BTN1, last1, lastEdge1, 1);
  pollBtn(BTN2, last2, lastEdge2, 2);
  pollBtn(BTN3, last3, lastEdge3, 3);
  delay(5);
}