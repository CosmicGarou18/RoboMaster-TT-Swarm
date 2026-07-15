#include <WiFi.h>
#include <Wire.h>

#include <esp_now.h>
#include <RMTT_TOF.h>
#include <RMTT_Libs.h>
#include <RMTT_RGB.h>

const int CLEAR_CM = 10; // increased from 80



RMTT_TOF      tof;
RMTT_Protocol sdk;
RMTT_Matrix   tt_matrix;
RMTT_RGB      LED;

uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

char msgBuffer[256];

void msgrecieve(const uint8_t* mac, const uint8_t* msg, int len){
  memcpy(msgBuffer, msg, len);
  msgBuffer[len] = '\0';
  String s = String(msgBuffer);
  Serial.println((char*)msgBuffer);
}

void diag(const char* msg) {
  Serial.println(msg);
  esp_now_send(BROADCAST, (uint8_t*)msg, strlen(msg));
}

void diag(const String& msg) {
  diag(msg.c_str());
}

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 26);
  Wire.setClock(400000);

  LED.Init();

  tof.Init();
  tof.SetMeasurementTimingBudget(30000);
  tof.StartContinuous();

  delay(5000);
  LED.SetRGB(110,60,560);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  diag("ESP-NOW ready");
  delay(6000);
  esp_now_register_recv_cb(msgrecieve);
}
void loop() {
  int mm = tof.ReadRangeContinuousMillimeters();
  int cm = mm / 10;

    if (cm == 0 || cm > CLEAR_CM) {}
    else {
      diag("danger :r");
    }
}