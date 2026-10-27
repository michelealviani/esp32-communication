
#include <WiFi.h>

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  

}

void loop(){
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  delay(10000);
}