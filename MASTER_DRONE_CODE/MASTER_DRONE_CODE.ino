#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_now.h>
#include <RMTT_Protocol.h>
#include <RMTT_TOF.h>
#include <RMTT_Libs.h>
#include <RMTT_RGB.h>

// --- Hardware Objects ---
RMTT_Protocol sdk;
WiFiUDP stateUDP;   // To capture the bridged telemetry stream
const int STATE_PORT = 8890;

// --- ESP-NOW Peer ---
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== INITIALIZING HARDWARE BRIDGE ===");

    // 1. Fire up the physical Serial link to the drone first
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);
    sdk.startUntilControl(); 

    // 2. Configure the ESP32 as an Access Point instead of trying to search for the Tello wirelessly
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RMTT_Swarm_Node", "12345678");
    Serial.println("AP Mode Spawned locally.");

    // 3. Initialize ESP-NOW 
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    // 4. Open the UDP port locally to catch the internal telemetry mirror
    stateUDP.begin(STATE_PORT);
    
    // Tell the drone processor via Serial to start mirroring UDP telemetry packets to the expansion slot
    Serial1.print("[TELLO] command");
    delay(200);
    Serial1.print("[TELLO] streamon");
    delay(200);
}

void loop() {
    // Listen for UDP data packages hitting port 8890 over the hardware bridge
    int packetSize = stateUDP.parsePacket();
    if (packetSize > 0) {
        char buffer[256];
        stateUDP.read(buffer, packetSize);
        buffer[packetSize] = '\0';
        
        // Transmit out to your other swarm modules instantly
        esp_now_send(broadcastAddress, (uint8_t*)buffer, strlen(buffer));
        Serial.printf("Bridged Telemetry Payload: %s\n", buffer);
    }
}