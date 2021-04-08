/**
 * Project: ping_alive.ino
 *
 * Description: ESP32 http client to get a web page as a remote ping alive notification.
 *
 * Author: Derek Chadwick
 * Date: 01/12/2020
 */

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#define USE_SERIAL Serial

WiFiMulti wifiMulti;



uint8_t blink;

void setup() 
{
  // Turn-off the 'brownout detector'
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  
    USE_SERIAL.begin(115200);
    
    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();
    
    pinMode(2, OUTPUT);      // set the LED pin mode
    blink = 0;
    
    for(uint8_t t = 4; t > 0; t--) 
    {
        USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    wifiMulti.addAP("xxxxx", "xxxxx");

}

void loop() 
{
    // wait for WiFi connection
    if((wifiMulti.run() == WL_CONNECTED)) {

        HTTPClient http;

        USE_SERIAL.print("[HTTP] begin...\n");
        
        //http.begin("https://www.example.com/a/check", ca); //HTTPS
        http.begin("http://www.example.com/checkout.html"); //HTTP

        USE_SERIAL.print("[HTTP] GET...from www.example.com\n");
        // start connection and send HTTP header
        int httpCode = http.GET();

        // httpCode will be negative on error
        if(httpCode > 0) 
        {
            // HTTP header has been send and Server response header has been handled
            USE_SERIAL.printf("[HTTP] GET... code: %d\n", httpCode);

            // file found at server
            if(httpCode == HTTP_CODE_OK) 
            {
                String payload = http.getString();
                USE_SERIAL.println(payload);
            }
        } 
        else 
        {
            USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
    }
    if (blink == 0) 
    {
      digitalWrite(2, HIGH);               // GET /H turns the LED on
      blink = 1;
    } else
    if (blink == 1) 
    {
      digitalWrite(2, LOW);                // GET /L turns the LED off
      blink = 0;
    }
    USE_SERIAL.printf("Delaying a while...\n");

    delay(1000);
}
