#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <RMTT_TOF.h>
#include <RMTT_Protocol.h>
#include <RMTT_RGB.h>

RMTT_TOF      tof;
RMTT_Protocol sdk;
RMTT_RGB      LED;

const int CLEAR_CM = 100; // increased from 80

char msgBuffer[256];
uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum class DroneType {
  None,
  Left,
  Right,
};

volatile DroneType droneType = DroneType::None; 
bool registrationPrinted = false;

void sendDroneCmd(const String cmd) {
  while (Serial1.available()) Serial1.read();
  Serial1.printf("[TELLO] %s", cmd);
  String cmdStr = String(cmd);
}

void diag(const char* msg) {
  Serial.println(msg);
  esp_now_send(BROADCAST, (uint8_t*)msg, strlen(msg));
}

void diag(const String& msg) {
  diag(msg.c_str());
}

// Single Master Receiver Callback
void masterReceiveHandler(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) return;
  
  memcpy(msgBuffer, data, len);
  msgBuffer[len] = '\0';
  String s = String(msgBuffer);

  DroneType t = DroneType::None;

  if (droneType == DroneType::None) {
    if (s.equals("l")) {
      droneType = DroneType::Left;
    }
    else if (s.equals("r")) {
      droneType = DroneType::Right;
    }
  }
  else {
    if (s.endsWith(":l")) {
      t = DroneType::Left;
    } else if (s.endsWith(":r")) {
      t = DroneType::Right;
    }
  
    if (droneType != t) {
      return;
    }

    msgBuff[len - 2] = 0x00;

    sendDroneCmd(msgBuff);
  }
}

void setup() {
  // 1. Establish Serial immediately
  Serial.begin(115200);
  
  // 2. Give the USB-to-PC connection time to breathe so prints aren't skipped
  delay(1500); 
  
  Serial.println("\n\n====================================");
  Serial.println("=== GROUND CONTROL TEST MODULE BOOT ===");
  Serial.println("====================================");
  Serial.flush();

  WiFi.mode(WIFI_STA);
  Serial.println("WiFi initialized to Station Mode.");
  Serial.flush();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW Initialization Failed!");
    Serial.flush();
    return;
  }
  
  esp_now_register_recv_cb(masterReceiveHandler);
  Serial.println("ESP-NOW: Receiver callback successfully linked.");
  Serial.flush();

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0; 
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: Failed to register Broadcast Peer!");
    Serial.flush();
    return;
  }
  Wire.begin(27, 26);
  Wire.setClock(400000);

  tof.Init();
  tof.SetMeasurementTimingBudget(30000);
  tof.StartContinuous();
  Serial1.begin(1000000, SERIAL_8N1, 23, 18);
  sdk.startUntilControl();
  delay(5000);
  
  LED.Init();
  LED.SetRGB(79, 125, 14);

  // Broadcast check-in signal
  esp_now_send(BROADCAST, (const uint8_t*)"here", 4);
  Serial.println("Sent initial 'here' discovery broadcast...");
  Serial.flush();
}

void loop() {
  if (droneType != DroneType::None && !registrationPrinted) {
    Serial.print("\n>>> SUCCESS: Registered dynamically as: ");
    if (droneType == DroneType::Left) Serial.println("LEFT SIDE DRONE <<<");
    if (droneType == DroneType::Right) Serial.println("RIGHT SIDE DRONE <<<");
    Serial.flush();
    
    registrationPrinted = true; 
  }

  int mm = tof.ReadRangeContinuousMillimeters();
  int cm = mm / 10;

  if (cm != 0 && cm > CLEAR_CM) {
    if (droneType == DroneType::Left) {
      diag("danger :l");
    }
    else if (droneType == DroneType::Right) {
      diag("danger :r");
    } 
  }

  // Yield control back to the background network processor
  delay(50);
}