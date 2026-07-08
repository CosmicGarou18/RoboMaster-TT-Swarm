#include <WiFi.h>
#include <esp_now.h>
#include <RMTT_TOF.h>

// ESP32 core 2.0.17 callback signature — do NOT use the esp_now_recv_info_t
// version, that's 3.x only.
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  char buf[251];
  int n = min(len, 250);
  memcpy(buf, data, n);
  buf[n] = '\0';
  Serial.printf("[%02X:%02X:%02X:%02X:%02X:%02X] %s\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], buf);
  // No blocking/delay here — callback runs in WiFi driver context.
}

const uint8_t BROADCAST[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// FIX: matches the rest of the project — pull-up, pressed = LOW
const int BUTTON_PIN = 34;
const unsigned long DEBOUNCE_MS = 50;
bool lastButtonState = HIGH;       // HIGH = not pressed, per project convention
unsigned long lastChangeMs = 0;

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED — halting");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("Receiver ready.");
  Serial.flush();

  pinMode(BUTTON_PIN, INPUT_PULLUP); // FIX: was INPUT_PULLDOWN
}

void loop() {
  bool current = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  // FIX: edge-triggered + debounced — fires exactly once per physical press,
  // not continuously for as long as the pin reads LOW
  if (current == LOW && lastButtonState == HIGH && (now - lastChangeMs) >= DEBOUNCE_MS) {
    Serial.println("DANGER");
    esp_now_send(BROADCAST, (uint8_t*)"danger", 6);
    lastChangeMs = now;
  }

  if (current != lastButtonState) {
    lastChangeMs = now;
  }
  lastButtonState = current;
}