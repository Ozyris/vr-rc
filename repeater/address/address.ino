#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    Serial.print("MAC bytes: ");
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", 
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {
    delay(1000);
}