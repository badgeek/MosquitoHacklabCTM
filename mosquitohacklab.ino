#define ENABLE_WIFI
//#define ENABLE_WIFI_AP
#define ENABLE_DEBUG

#include "MPU9250.h"

#ifdef ENABLE_WIFI
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#endif

#include <OSCMessage.h>


#define WIFI_AP_SSID "MOSQUITOLAB"
#define WIFI_AP_PASSWORD "thereisnospoon"
#define WIFI_CL_SSID "RUMAH"
#define WIFI_CL_PASSWORD "rumah4321"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
IPAddress broadcastIp;
WiFiUDP Udp;

void setup() {

#ifdef ENABLE_WIFI_AP
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  delay(500);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_CL_SSID, WIFI_CL_PASSWORD);
  delay(500);
#endif

#ifdef ENABLE_DEBUG
  Serial.begin(115200);
#endif

  broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();
  Udp.begin(8888);

#ifdef ENABLE_DEBUG
  Serial.println(broadcastIp);
#endif

#ifdef ENABLE_DEBUG
  while (!Serial) {}
#endif

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
#ifdef ENABLE_DEBUG
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
#endif
    while (1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  //  // display the data
  //  Serial.print(IMU.getAccelX_mss(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getAccelY_mss(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getAccelZ_mss(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroX_rads(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroY_rads(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getGyroZ_rads(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagX_uT(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagY_uT(),6);
  //  Serial.print("\t");
  //  Serial.print(IMU.getMagZ_uT(),6);
  //  Serial.print("\t");
  //  Serial.println(IMU.getTemperature_C(),6);

  OSCMessage msg("/test");
  msg.add(IMU.getAccelX_mss());
  msg.add(IMU.getAccelY_mss());
  msg.add(IMU.getAccelZ_mss());

  Udp.beginPacket(broadcastIp, 8888);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  delay(100);
}
