#include <Wire.h>
#include <RMTT_Libs.h>

// ====== VALUES YOU SET ======
const int16_t  RC_FORWARD_VALUE = 40;    // -100..100, fb channel — the value you're characterizing THIS run
const uint16_t STOP_DISTANCE_MM = 400;   // trigger distance
const uint32_t MAX_FLIGHT_MS    = 7000; // abort if TOF never fires
// =============================

// Your own project notes: this sensor caps ~1200mm indoors. Set above that and it silently never triggers.
#if STOP_DISTANCE_MM > 1200
  #error "STOP_DISTANCE_MM > 1200mm — outside the sensor's documented range. You will hit the wall."
#endif

RMTT_TOF      tof;
RMTT_Protocol sdk;

uint32_t takeoffTime = 0;
bool     stopped = false;

void sendDroneCmd(const char* cmd) {
  while (Serial1.available()) Serial1.read();
  Serial1.printf("[TELLO] %s", cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== FRONT-TOF TIME-IN-AIR TEST ===");

  Wire.begin(27, 26);
  Wire.setClock(400000);

  if (!tof.Init()) {
    Serial.println("TOF init FAILED — check wiring/mount before flying. Halting.");
    while (true) delay(1000);
  }
  tof.setTimeout(500);
  tof.StartContinuous();
  Serial.println("Forward TOF ready.");

  delay(5000); // drone boot wait — non-negotiable per your boot order

  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  Serial.println("Serial1 up. Waiting for handshake + button press...");
  sdk.startUntilControl(); // blocks on 'ETT ok' + physical button — this is intentional, not a hang

  Serial.println("Handshake confirmed. Taking off in 2s...");
  delay(2000);

  sendDroneCmd("takeoff");
  delay(6000); // post-takeoff settle before trusting anything

  takeoffTime = millis();
  Serial.printf("TAKEOFF. Timer started. rc forward = %d.\n", RC_FORWARD_VALUE);
}

void loop() {
  if (stopped) return;

  while (Serial1.available()) Serial1.read();

  uint16_t distMM = tof.ReadRangeContinuousMillimeters();
  bool valid = !tof.TimeoutOccurred() && distMM != 65535 && distMM != 0;
  uint32_t elapsedMS = millis() - takeoffTime;

  if (sdk.)

  if (valid && distMM <= STOP_DISTANCE_MM) {
    sendDroneCmd("rc 0 0 0 0");
    delay(300);
    sendDroneCmd("land");
    stopped = true;
    Serial.println("=== OBSTACLE DETECTED ===");
    Serial.printf("Distance at stop: %d mm (threshold %d mm)\n", distMM, STOP_DISTANCE_MM);
    Serial.printf("RC value: %d\n", RC_FORWARD_VALUE);

    while (true) {
      Serial.printf("TIME IN AIR: %.3f s\n", elapsedMS / 1000.0);
      delay(500);
    }

    return;
  }

  if (elapsedMS > MAX_FLIGHT_MS) {
    sendDroneCmd("rc 0 0 0 0");
    delay(300);
    sendDroneCmd("land");
    stopped = true;
    Serial.println("=== TIMEOUT, NO DETECTION — DISCARD THIS RUN ===");
    Serial.println("Either STOP_DISTANCE_MM unreachable in your space, or TOF isn't facing the obstacle.");
    return;
  }

  char cmd[20];
  snprintf(cmd, sizeof(cmd), "rc 0 %d 0 0", RC_FORWARD_VALUE);
  sendDroneCmd(cmd); // this also serves as the heartbeat — resets the 15s auto-land clock every cycle

  delay(50); // ~20Hz, matches your existing control loop convention
}