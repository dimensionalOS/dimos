// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference firmware for Esp32GestureTeleop: a distance sensor on any ESP32 becomes a
// one-gesture controller that advertises over BLE. It knows nothing about any robot.
//
// Gesture: something (a hand) stays between MIN_CM and MAX_CM for HOLD_MS. The board then
// alternates gesture A, B, A... and bumps a counter. It must be gone for REARM_MS before the
// next gesture, so holding still never repeats.
//
// Advertisement: name DEVICE_NAME, manufacturer data FF FF <state> <seq> <gesture_id> <cm> <count>
//   state       0/1, flipped by the button on BTN_PIN (LED_PIN shows it)
//   seq         increments on every advertisement update
//   gesture_id  0 = none yet, 1 = A, 2 = B
//   cm          latest distance, 0 = no echo
//   count       increments once per gesture, wraps at 255
//
// Every setting below can be changed from the build command instead of editing the file, e.g.
//   arduino-cli compile --fqbn esp32:esp32:esp32 \
//     --build-property "build.extra_flags=-DTRIG_PIN=12 -DECHO_PIN=14" esp32_gesture
//
// The default sensor is an HC-SR04-style ultrasonic module (VCC->3V3, GND->GND, TRIG->TRIG_PIN,
// ECHO->ECHO_PIN). For another distance sensor, replace readCm(); nothing else depends on it.

#include <BLEAdvertising.h>
#include <BLEDevice.h>

// ---- Board pins ----
#ifndef BTN_PIN
#define BTN_PIN 0  // BOOT button on most dev boards
#endif
#ifndef LED_PIN
#define LED_PIN 2  // set to -1 if the board has no LED
#endif
#ifndef TRIG_PIN
#define TRIG_PIN 5
#endif
#ifndef ECHO_PIN
#define ECHO_PIN 18
#endif

// ---- BLE ----
#ifndef DEVICE_NAME
#define DEVICE_NAME "dimos-gesture"
#endif
#ifndef ADV_UPDATE_MS
#define ADV_UPDATE_MS 100  // how often the advertisement is refreshed
#endif

// ---- Sensor timing ----
#ifndef PING_MS
#define PING_MS 60  // time between distance readings
#endif
#ifndef ECHO_TIMEOUT_US
#define ECHO_TIMEOUT_US 30000  // no echo after this = nothing in range
#endif
#ifndef TRIGGER_PULSE_US
#define TRIGGER_PULSE_US 500  // 10 is enough for an HC-SR04; some clones (TK50) need ~500
#endif

// ---- Gesture ----
#ifndef MIN_CM
#define MIN_CM 5.0
#endif
#ifndef MAX_CM
#define MAX_CM 40.0
#endif
#ifndef HOLD_MS
#define HOLD_MS 500  // in range this long -> gesture
#endif
#ifndef REARM_MS
#define REARM_MS 500  // out of range this long -> ready for the next gesture
#endif
#ifndef MISS_TOLERANCE
#define MISS_TOLERANCE 3  // consecutive missed readings before the hand counts as gone
#endif
#ifndef DEBOUNCE_MS
#define DEBOUNCE_MS 250  // button
#endif

BLEAdvertising *adv;
uint8_t state = 0, seq = 0, gestureId = 0, cmByte = 0, count = 0;
bool present = false, armed = true;
int misses = 0, lastBtn = HIGH;
unsigned long presentSince = 0, absentSince = 0, lastUpdate = 0, lastPress = 0, lastPing = 0;

float readCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIGGER_PULSE_US);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  return us ? us / 58.0 : 0;
}

void setLed(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void setAdvData() {
  BLEAdvertisementData d;
  d.setFlags(0x06);
  d.setName(DEVICE_NAME);
  String m;
  m += (char)0xFF;
  m += (char)0xFF;
  m += (char)state;
  m += (char)seq;
  m += (char)gestureId;
  m += (char)cmByte;
  m += (char)count;
  d.setManufacturerData(m);
  adv->setAdvertisementData(d);
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  setLed(false);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  BLEDevice::init(DEVICE_NAME);
  adv = BLEDevice::getAdvertising();
  adv->setMinInterval(0x50);
  adv->setMaxInterval(0xA0);
  setAdvData();
  adv->start();
  Serial.printf("%s up, MAC %s\n", DEVICE_NAME, BLEDevice::getAddress().toString().c_str());
}

void loop() {
  unsigned long now = millis();

  int b = digitalRead(BTN_PIN);
  if (b == LOW && lastBtn == HIGH && now - lastPress >= DEBOUNCE_MS) {
    lastPress = now;
    state = !state;
    setLed(state);
    Serial.printf("state %u\n", state);
  }
  lastBtn = b;

  if (now - lastPing >= PING_MS) {
    lastPing = now;
    float cm = readCm();
    cmByte = (cm > 0 && cm < 255) ? (uint8_t)cm : 0;
    bool inBand = cm >= MIN_CM && cm <= MAX_CM;
    misses = inBand ? 0 : misses + 1;
    bool nowPresent = inBand || (present && misses < MISS_TOLERANCE);
    if (nowPresent && !present) presentSince = now;
    if (!nowPresent && present) absentSince = now;
    present = nowPresent;
    if (!present && !armed && now - absentSince >= REARM_MS) armed = true;
    if (present && armed && now - presentSince >= HOLD_MS) {
      armed = false;
      gestureId = (gestureId == 1) ? 2 : 1;
      count++;
      Serial.printf("gesture %c (#%u, %.0f cm)\n", gestureId == 1 ? 'A' : 'B', count, cm);
    }
  }

  if (now - lastUpdate >= ADV_UPDATE_MS) {
    lastUpdate = now;
    seq++;
    adv->stop();
    setAdvData();
    adv->start();
  }
}
