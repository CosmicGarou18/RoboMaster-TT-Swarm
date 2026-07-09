#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <RMTT_Protocol.h>
#include <RMTT_TOF.h>
#include <RMTT_Libs.h>
#include <RMTT_RGB.h>

const uint8_t BROADCAST[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ====================================================================
// CLASS: PIDController (Pure Math Logic)
// ====================================================================
class PIDController {
private:
  float kp, ki, kd;
  float integralLimit;
  float integral;
  float lastError;

public:
  PIDController(float p, float i, float d, float iLimit) 
    : kp(p), ki(i), kd(d), integralLimit(iLimit), integral(0.0f), lastError(0.0f) {}

  void reset() {
    integral = 0.0f;
    lastError = 0.0f;
  }

  float compute(float error, float dtSec) {
    if (dtSec > 0) {
      integral += error * dtSec;
      integral = constrain(integral, -integralLimit, integralLimit);
    }
    float derivative = (dtSec > 0) ? (error - lastError) / dtSec : 0.0f;
    lastError = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
  }
};

// ====================================================================
// CLASS: DroneController (Pure Movement/Target Tracking Layer)
// ====================================================================
class DroneController {
private:
  PIDController pid;
  float targetDistMeters;
  const float outScale = 65.0f; // Scale factor mapping meter error to RC
  const int maxPitchCmd = 35;   // Hard limit clamp for trial safety

public:
  DroneController() : pid(8f, 0.05f, 0.25f, 4.0f), targetDistMeters(0.60f) {}

  void setTargetDistanceMm(int targetMm) {
    targetDistMeters = targetMm / 1000.0f;
  }

  void resetController() {
    pid.reset();
  }

  int updateTracking(int currentDistMm, float dtSec) {
    float currentDistMeters = currentDistMm / 1000.0f;
    
    // Error tracking: positive error means drone is too far back (must fly forward)
    float error = currentDistMeters - targetDistMeters;
    
    float pidOutput = pid.compute(error, dtSec);
    int rcPitchCmd = (int)(pidOutput * outScale);
    rcPitchCmd = constrain(rcPitchCmd, -maxPitchCmd, maxPitchCmd);

    // Issue the tracking movement directly to the flight hardware
    char rcBuf[32];
    snprintf(rcBuf, sizeof(rcBuf), "rc 0 %d 0 0", rcPitchCmd);
    sendDroneCmd(rcBuf, true); 

    return rcPitchCmd; // Returns the applied value for external logging/telemetry
  }
};

void diag(const char* msg) {
  Serial.println(msg);
  esp_now_send(BROADCAST, (uint8_t*)msg, strlen(msg));
}
void diag(const String& msg) { diag(msg.c_str()); }

void sendDroneCmd(const char* cmd, bool silent = false) {
  while (Serial1.available()) Serial1.read();
  Serial1.printf("[TELLO] %s", cmd);
  if (!silent) diag(cmd);
}

RMTT_RGB LED;
RMTT_Protocol sdk;
RMTT_TOF tof;

// ---- Trial config ----
const int   RC_PCT              = 20;
const int   SAFE_DIST_MM        = 200;
const int   CONSEC_REQUIRED     = 2;
const int   TRIALS_TARGET       = 10;
const unsigned long TAKEOFF_DELAY_MS = 1000; // FIX: time to walk away after button press

int consecUnsafe = 0;
int trialCount = 0;
float velocitySumCms = 0.0f; // FIX: now accumulating cm/s, not m/s

int x0 = 0;
unsigned long t0 = 0;
int lastDist = -1;
unsigned long lastTickMs = 0;

unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_MS = 3000;
unsigned long lastDecision = 0;
const unsigned long DECISION_MS = 150;

enum State { WAITING_FOR_BUTTON, FLYING };
State state = WAITING_FOR_BUTTON;

// FIX: velocities now in cm/s throughout
void sendTelemetry(int distMm, float instVelCms, float avgVelCms, unsigned long elapsedMs) {
  char buf[100];
  snprintf(buf, sizeof(buf), "RC:%d D:%d T:%lu IV:%.2f AV:%.2f",
           RC_PCT, distMm, elapsedMs, instVelCms, avgVelCms);
  esp_now_send(BROADCAST, (uint8_t*)buf, strlen(buf));
  Serial.println(buf);
}

void doLandSequence(float finalAvgVelCms) {
  sendDroneCmd("rc 0 -50 0 0");
  delay(500);
  sendDroneCmd("rc 0 0 0 0");
  delay(300);
  sendDroneCmd("land");
  state = WAITING_FOR_BUTTON;
  consecUnsafe = 0;
  lastDist = -1;

  trialCount++;
  velocitySumCms += finalAvgVelCms;

  char summary[100];
  snprintf(summary, sizeof(summary), "TRIAL %d/%d DONE RC:%d AVG_V:%.2f cm/s",
           trialCount, TRIALS_TARGET, RC_PCT, finalAvgVelCms);
  diag(summary);
  LED.SetRGB(255, 165, 0);

  if (trialCount >= TRIALS_TARGET) {
    float overallAvg = velocitySumCms / trialCount;
    char final[100];
    snprintf(final, sizeof(final), "ALL TRIALS DONE RC:%d OVERALL_AVG_V:%.2f cm/s over %d trials",
             RC_PCT, overallAvg, trialCount);
    diag(final);
    trialCount = 0;
    velocitySumCms = 0.0f;
  }
}

void startTrial() {
  // FIX: countdown delay so you can move away — sent over ESP-NOW and printed locally
  char msg[40];
  for (int s = TAKEOFF_DELAY_MS / 1000; s > 0; s--) {
    snprintf(msg, sizeof(msg), "TAKEOFF IN %d...", s);
    diag(msg);
    delay(1000);
  }
  diag("TAKING OFF NOW");

  int firstDist = tof.ReadRangeContinuousMillimeters();
  while (firstDist == 0) {
    delay(20);
    firstDist = tof.ReadRangeContinuousMillimeters();
  }
  x0 = firstDist;
  t0 = millis();
  lastDist = x0;
  lastTickMs = t0;
  consecUnsafe = 0;

  sendDroneCmd("takeoff");
  delay(6000);

  LED.SetRGB(36, 153, 12);
  sendDroneCmd("rc 0 20 0 0"); // keep in sync with RC_PCT
  lastHeartbeat = millis();
  state = FLYING;
  diag("TRIAL STARTED");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 26);
  Wire.setClock(400000);

  tof.Init();
  tof.SetMeasurementTimingBudget(30000);
  tof.StartContinuous();

  pinMode(34, INPUT_PULLUP);
  delay(5000);

  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  sdk.startUntilControl();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (true) delay(1000);
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  LED.SetRGB(255, 165, 0);
  diag("READY — position drone facing wall, press button to start trial");
}

void hover() {
  DroneController c;
  c.setTargetDistanceMm(sdk.)
}

void loop() {
  while (Serial1.available()) Serial1.read();

  if (state == WAITING_FOR_BUTTON) {
    if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
      sendDroneCmd("battery?", true);
      lastHeartbeat = millis();
    }
    if (digitalRead(34) == 0) {
      delay(50);
      while (digitalRead(34) == 0) delay(10);
      startTrial(); // countdown now happens inside here, before takeoff
    }
    return;
  }

  if (millis() - lastDecision < DECISION_MS) return;
  lastDecision = millis();

  int dist = tof.ReadRangeContinuousMillimeters();
  unsigned long now = millis();

  if (dist == 0) {
    if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
      sendDroneCmd("rc 0 20 0 0", true);
      lastHeartbeat = millis();
    }
    return;
  }

  float dtSec      = (now - lastTickMs) / 1000.0f;
  // FIX: cm/s instead of m/s — (mm difference) / 10 = cm, divided by dtSec
  float instVelCms = (dtSec > 0) ? (lastDist - dist) / 10.0f / dtSec : 0;
  float elapsedSec = (now - t0) / 1000.0f;
  float avgVelCms  = (elapsedSec > 0) ? (x0 - dist) / 10.0f / elapsedSec : 0;

  sendTelemetry(dist, instVelCms, avgVelCms, now - t0);

  lastDist = dist;
  lastTickMs = now;

  if (dist < SAFE_DIST_MM) {
    consecUnsafe++;
    if (consecUnsafe >= CONSEC_REQUIRED) {
      doLandSequence(avgVelCms);
      return;
    }
  } else {
    consecUnsafe = 0;
  }

  if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
    sendDroneCmd("rc 0 20 0 0", true);
    lastHeartbeat = millis();
  }
}