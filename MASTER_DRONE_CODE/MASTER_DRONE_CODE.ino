#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <RMTT_Protocol.h>
#include <RMTT_TOF.h>
#include <RMTT_Libs.h>

const unsigned long ROAM_MS     = 60000; // 1 minute
const unsigned long WARNING_MS  =  5000;
const unsigned long DECISION_MS =   150;
const int CLEAR_CM = 100; // increased from 80

bool airborne       = false;
bool warningShown   = false;
unsigned long missionStart = 0;
unsigned long lastDecision = 0;
bool isTurning      = false;
String lastCmd      = "";

bool matrixFlashOn      = false;
unsigned long lastFlash = 0;
const unsigned long FLASH_MS = 300;

RMTT_TOF      tof;
RMTT_Protocol sdk;
RMTT_Matrix   tt_matrix;

uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ── MATRIX PATTERNS ─────────────────────────────────
// Buffer is 128 bytes, 16x8 grid, each pixel = [red, blue]

uint8_t matrix_off[128];
uint8_t matrix_red[128];

// Arrow pointing UP (forward)
uint8_t matrix_up[128] = {
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 255,0, 255,0, 255,0, 0,0, 0,0,
  0,0, 0,0, 255,0, 0,0, 255,0, 0,0, 255,0, 0,0,
  0,0, 255,0, 0,0, 0,0, 255,0, 0,0, 0,0, 255,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0,
};

// Arrow pointing RIGHT (turning CW)
uint8_t matrix_right[128] = {
  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0,
  255,0, 255,0, 255,0, 255,0, 255,0, 255,0, 255,0, 0,0,
  255,0, 255,0, 255,0, 255,0, 255,0, 255,0, 255,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0,
};

// Arrow pointing DOWN (braking/reversing)
uint8_t matrix_down[128] = {
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 255,0, 0,0, 0,0, 255,0, 0,0, 0,0, 255,0,
  0,0, 0,0, 255,0, 0,0, 255,0, 0,0, 255,0, 0,0,
  0,0, 0,0, 0,0, 255,0, 255,0, 255,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 255,0, 0,0, 0,0, 0,0,
  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0,
};

void buildMatrices() {
  for (int i = 0; i < 128; i += 2) {
    matrix_red[i]     = 255;
    matrix_red[i + 1] = 0;
    matrix_off[i]     = 0;
    matrix_off[i + 1] = 0;
  }
}

void diag(const char* msg) {
  Serial.println(msg);
  esp_now_send(BROADCAST, (uint8_t*)msg, strlen(msg));
}

void diag(const String& msg) {
  diag(msg.c_str());
}

void sendDroneCmd(const char* cmd, bool silent = false) {
  while (Serial1.available()) Serial1.read();
  Serial1.printf("[TELLO] %s", cmd);
  String cmdStr = String(cmd);
  if (!silent && cmdStr != lastCmd) {
    diag(String("CMD: ") + cmd);
    lastCmd = cmdStr;
  }
}

void setup() {
  Serial.begin(115200);

  buildMatrices();

  Wire.begin(27, 26);
  Wire.setClock(400000);

  tt_matrix.Init(127);
  tt_matrix.SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW, RMTT_MATRIX_LED_ON);
  tt_matrix.SetAllPWM((uint8_t*)matrix_off);

  tof.Init();
  tof.SetMeasurementTimingBudget(30000);
  tof.StartContinuous();

  delay(5000);

  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  sdk.startUntilControl();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  diag("ESP-NOW ready");
  sendDroneCmd("battery?")
  sendDroneCmd("takeoff");
  delay(6000);

  missionStart = millis();
  airborne     = true;
  diag("Mission started");
}

void loop() {
  while (Serial1.available()) Serial1.read();

  if (!airborne) return;

  unsigned long elapsed = millis() - missionStart;

  // A. TIME EXPIRED → LAND
  if (elapsed >= ROAM_MS + WARNING_MS) {
    diag("TIME EXPIRED - landing");
    sendDroneCmd("rc 0 0 0 0");
    delay(300);
    tt_matrix.SetAllPWM((uint8_t*)matrix_off);
    sendDroneCmd("land");
    airborne = false;
    return;
  }

  // B. WARNING PHASE — flash red matrix
  if (elapsed >= ROAM_MS) {
    if (!warningShown) {
      warningShown = true;
      diag("WARNING: 5s to land");
      sendDroneCmd("rc 0 0 0 0");
    }
    if (millis() - lastFlash >= FLASH_MS) {
      lastFlash = millis();
      matrixFlashOn = !matrixFlashOn;
      tt_matrix.SetAllPWM(matrixFlashOn ? (uint8_t*)matrix_red : (uint8_t*)matrix_off);
    }
    return;
  }

  // C. ROAM PHASE
  if (!warningShown && !isTurning && (millis() - lastDecision >= DECISION_MS)) {
    lastDecision = millis();

    int mm = tof.ReadRangeContinuousMillimeters();
    int cm = mm / 10;

    if (cm == 0 || cm > CLEAR_CM) {
      sendDroneCmd("rc 0 40 0 0");
      if (lastCmd != "rc 0 40 0 0") {
        diag("FORWARD");
        tt_matrix.SetAllPWM((uint8_t*)matrix_up);
      }
    } else {
      diag("BLOCKED " + String(cm) + "cm - TURNING");
      isTurning = true;

      // Brake
      tt_matrix.SetAllPWM((uint8_t*)matrix_down);
      sendDroneCmd("rc 0 -70 0 0");
      delay(750);

      // Stabilise
      sendDroneCmd("rc 0 0 0 0");
      delay(750);

      // Turn
      tt_matrix.SetAllPWM((uint8_t*)matrix_right);
      sendDroneCmd("cw 90");
      delay(2000);

      isTurning = false;
      diag("Turn complete");
    }
  }
}