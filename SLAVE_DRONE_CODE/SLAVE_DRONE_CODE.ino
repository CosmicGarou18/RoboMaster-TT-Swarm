#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#include <RMTT_Protocol.h>

char msgBuffer[256];
uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile enum {
  NONE,
  LEFT,
  RIGHT,
} type = NONE;

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) return;
  memcpy(msgBuffer, data, len);
  msgBuffer[len] = '\0';
  String s = String(msgBuffer);

  if (s.equals("l")) {
    type = LEFT;
  }
  else if (s.equals("r")) {
    type = RIGHT;
  }
}

void handleCommands(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) return;
  memcpy(msgBuffer, data, len);
  msgBuffer[len] = '\0';
  String s = String(msgBuffer);
  Serial.println(s); 
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== GROUND STATION BOOT ===");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW: FAILED");
    return;
  }

  interrupts();
  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP-NOW: ready, listening...");

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_now_send(BROADCAST, (const uint8_t*)"here", 4);
  Serial.println("Sent 'here' command");

  while (type == NONE) {}
  Serial.println("Registered");
  esp_now_register_recv_cb(handleCommands);
}

void loop() {
}