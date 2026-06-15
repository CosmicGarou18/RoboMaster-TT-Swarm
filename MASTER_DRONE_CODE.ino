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

bool airborne       = false;
bool warningShown   = false;
unsigned long missionStart = 0;
unsigned long lastDecision = 0;
bool isTurning      = false;

RMTT_TOF      tof;
RMTT_Protocol sdk;

uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void diag(const char* msg) {
  Serial.println(msg);
  esp_now_send(BROADCAST, (uint8_t*)msg, strlen(msg));
}

void diag(const String& msg) {
  diag(msg.c_str());
}

void sendDroneCmd(const char* cmd) {
  Serial1.printf("[TELLO] %s", cmd);
  diag(String("CMD: ") + cmd);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(27, 26);
  Wire.setClock(400000);
  tof.Init();
  tof.SetMeasurementTimingBudget(30000);
  tof.StartContinuous();

  delay(5000);

  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  sdk.startUntilControl();

  // Force WiFi to station mode and lock to channel 1
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  diag("ESP-NOW ready");

  sendDroneCmd("takeoff");
  delay(6000);

  missionStart = millis();
  airborne     = true;
  diag("Mission started");
}

void loop() {
  if (!airborne) return;

  unsigned long elapsed = millis() - missionStart;

  if (elapsed >= ROAM_MS + WARNING_MS) {
    diag("TIME EXPIRED - landing");
    sendDroneCmd("rc 0 0 0 0");
    delay(300);
    sendDroneCmd("land");
    airborne = false;
    return;
  }

  if (elapsed >= ROAM_MS) {
    if (!warningShown) {
      warningShown = true;
      diag("WARNING: hovering");
      sendDroneCmd("rc 0 0 0 0");
    }
    return;
  }

  if (!warningShown && !isTurning && (millis() - lastDecision >= DECISION_MS)) {
    lastDecision = millis();

    int mm = tof.ReadRangeContinuousMillimeters();
    int cm = mm / 10;

    if (cm == 0 || cm > CLEAR_CM) {
      diag("FORWARD");
      sendDroneCmd("rc 0 40 0 0");
    } else {
      diag("BLOCKED " + String(cm) + "cm - TURNING");
      isTurning = true;
      sendDroneCmd("rc 0 -60 0 0");
      delay(500);
      sendDroneCmd("rc 0 0 0 0");
      delay(300);
      sendDroneCmd("cw 90");
      delay(2000);
      isTurning = false;
      diag("Turn complete");
    }
  }
}