#include <WiFi.h>
#include <WiFiUdp.h>

#include <RMTT_Protocol.h>

const char SSID[] = "MASTER-DRONE";
const char PASS[] = "";

WiFiUDP listener;
RMTT_Protocol sdk;

void setup() {
    Serial.begin(9600);
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);

    sdk.startUntilControl();

    WiFi.softAP(SSID, PASS);    
    listener.begin(8890);
}

void loop() {
    if (!listener.parsePacket()) {
        return;
    }

    String s = listener.readStringUntil('\n');
    Serial.println(s);
    delay(1000);
}
