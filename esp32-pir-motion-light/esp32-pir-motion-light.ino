/*
  
  Project: ESP32 Surveillance System
  Author: Derek Chadwick
  
 

  Description: Uses ESP32 Devkit C 1.0 to switch on a security light on PIR motion detect,
               then uses udp comms to send notifications to log server.
               
               


*/

#include "WiFi.h"
#include "esp_timer.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include <StringArray.h>
#include <SPIFFS.h>
#include <FS.h>
#include <EEPROM.h>            // read and write from flash memory
#include <Preferences.h>
#include <WiFiUdp.h>

Preferences prefs;
WiFiUDP udp;

// Replace with your network credentials
const char* ssid = "xxxxx";
const char* password = "xxxxx";
const char* log_server_ip = "192.168.1.99";
const int log_port = 8989;
 
boolean switchLEDS = false;
boolean ledsOn = false;
int detectNumber = 0;
int connectCount = 0;
boolean connected = false;

// define the number of bytes you want to access
#define EEPROM_SIZE 1


void setup() 
{
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Serial port for debugging purposes
  Serial.begin(115200);
  //DEPRECATED: system_get_chip_id(); NOT AVAILABLE IN ESP32 SDK, Is just the MAC address anyway.
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while ((WiFi.status() != WL_CONNECTED) and (connectCount < 60)) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    connectCount = connectCount + 1;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    connected = true;
    udp.begin(WiFi.localIP(), log_port);
  }
  
  prefs.begin("lights", false);
  detectNumber = prefs.getUInt("counter", 0);
  
  // Print ESP32 Local IP Address
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
  Serial.print("APMAC: ");
  Serial.println(WiFi.softAPmacAddress());
  
  // Set LED pins to output.
  
  pinMode(2, OUTPUT);  // On-board blue LED.
  pinMode(27, INPUT);  // PIR signal input, active low because of 47k ohm pullup resistor.
  pinMode(26, OUTPUT); // Signal to gate of MOSFET to turn on security device.
  
  digitalWrite(2, LOW);
  digitalWrite(26, LOW);

}

void loop() 
{
  if (digitalRead(27) == HIGH) // PIR input signal, active high.
  {
    
    digitalWrite(26, HIGH);
    digitalWrite(2, HIGH);
    // TODO: send notification to log server.
    detectNumber = detectNumber + 1;
    prefs.putUInt("counter", detectNumber);
    Serial.printf("PIR motion detected: %u\n", detectNumber);
    send_notification();
    delay(10000);
  }
  else
  {
    digitalWrite(26, LOW);
    digitalWrite(2, LOW);
  }
  
}

void send_notification()
{
   //UDP client comms.  
   if (connected)
   {
       //send message to log server.
       udp.beginPacket(log_server_ip, log_port);
       udp.printf("PIR trigger count: %u");
       udp.endPacket();
       Serial.printf("PIR trigger count sent.\n");
   }
}

void flipled()
{
    Serial.println("Switching LEDS...");
    if (ledsOn)
    {
       digitalWrite(4, LOW);
       //digitalWrite(33, HIGH);     
    }
    else
    {
       digitalWrite(4, HIGH);
       //digitalWrite(33, LOW); 
    }
    ledsOn = !ledsOn;
  
}
