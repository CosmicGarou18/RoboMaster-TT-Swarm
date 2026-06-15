#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

char msgBuffer[256];

void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0 || len > 250) return;
  memcpy(msgBuffer, data, len);
  msgBuffer[len] = '\0';
  Serial.printf("[%02X:%02X:%02X:%02X:%02X:%02X] %s\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    msgBuffer);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== GROUND STATION BOOT ===");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onReceive);
    Serial.println("ESP-NOW: ready, listening...");
  } else {
    Serial.println("ESP-NOW: FAILED");
  }
  interrupts();
}

void loop() {
}