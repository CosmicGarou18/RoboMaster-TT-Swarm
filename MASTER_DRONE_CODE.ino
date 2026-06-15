#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <RMTT_Protocol.h>
#include <RMTT_TOF.h>
#include "esp_wifi.h"

const unsigned long ROAM_MS     = 25000;
const unsigned long WARNING_MS  =  5000;
const unsigned long DECISION_MS =   150;
const int CLEAR_CM = 100;

bool airborne      = false;
bool warningShown  = false;
bool isTurning     = false;
unsigned long missionStart = 0;
unsigned long lastDecision = 0;

RMTT_TOF      tof;
RMTT_Protocol sdk;

uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ─── helpers ────────────────────────────────────────────────────────────────

void drainSerial1() {
  while (Serial1.available()) Serial1.read();
}

void diag(const char* msg) {
  Serial.println(msg);
  esp_now_send(BROADCAST, (uint8_t*)msg, strlen(msg));
}

void diag(const String& msg) {
  diag(msg.c_str());
}

void sendDroneCmd(const char* cmd, unsigned long waitMs = 0) {
  drainSerial1();
  Serial1.printf("[TELLO] %s", cmd);
  diag(String("CMD: ") + cmd);
  if (waitMs > 0) delay(waitMs);
}

// ─── setup ──────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  Wire.begin(27, 26);
  Wire.setClock(400000);
  tof.Init();
  tof.SetMeasurementTimingBudget(30000);
  tof.StartContinuous();

  delay(5000);

  // Handshake first — do NOT touch WiFi before this
  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  sdk.startUntilControl();

  // ESP-NOW init after handshake — do NOT call WiFi.mode() here
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  diag("ESP-NOW ready");

  sendDroneCmd("takeoff", 6000);

  missionStart = millis();
  airborne     = true;
  diag("Mission started");
}

// ─── loop ───────────────────────────────────────────────────────────────────

void loop() {
  drainSerial1();

  if (!airborne) return;

  unsigned long elapsed = millis() - missionStart;

  // Phase 3: land
  if (elapsed >= ROAM_MS + WARNING_MS) {
    diag("TIME EXPIRED - landing");
    sendDroneCmd("rc 0 0 0 0", 300);
    sendDroneCmd("land", 0);
    airborne = false;
    return;
  }

  // Phase 2: hover warning
  if (elapsed >= ROAM_MS) {
    if (!warningShown) {
      warningShown = true;
      diag("WARNING: hovering");
      sendDroneCmd("rc 0 0 0 0", 0);
    }
    return;
  }

  // Phase 1: obstacle avoidance
  if (!isTurning && (millis() - lastDecision >= DECISION_MS)) {
    lastDecision = millis();

    int cm = tof.ReadRangeContinuousMillimeters() / 10;

    if (cm == 0 || cm > CLEAR_CM) {
      diag("FORWARD");
      sendDroneCmd("rc 0 40 0 0", 0);
    } else {
      diag("BLOCKED " + String(cm) + "cm - TURNING");
      isTurning = true;
      sendDroneCmd("rc 0 -60 0 0", 500);  // brake
      sendDroneCmd("rc 0 0 0 0",   300);  // stabilise
      sendDroneCmd("cw 90",       2500);  // rotate — bumped to 2500ms
      isTurning = false;
      diag("Turn complete");
    }
  }
}