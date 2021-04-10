/*
  
  Project: ESP32 Surveillance System
  Author: Derek Chadwick
  
  Based on code by Rui Santos  https://RandomNerdTutorials.com/esp32-cam-take-photo-display-web-server/
  
  IMPORTANT!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  

  Description: Takes a photo on PIR motion detect or on request from a web client then saves the photos to SD card
               and displays them in a web page.
               TODO: Implement painlessMesh so devices can communicate with an edge controller that sends notifications
                     by email or pushbullet. Also implement photo upload via web interface.
               
  NOTE: Pull-up resistors.
  GPIO4,12,13,14,15,16 used for SD card have 47k pullup resistors connected to the 3V3 rail.

*/

#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <ESPAsyncWebServer.h>
#include <StringArray.h>
#include <SPIFFS.h>
#include <FS.h>
//#include "SD_MMC.h"
#include <EEPROM.h>            // read and write from flash memory
//#include <WiFiMulti.h>

#include "webpage.h"

// Replace with your network credentials
const char* ssid = "xxxxx";
const char* password = "xxxxx";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

boolean takeNewPhoto = false;
boolean switchLEDS = false;
boolean ledsOn = false;
int pictureNumber = 0;
int connectCount = 0;
int delayCount = 0;

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"
// define the number of bytes you want to access
#define EEPROM_SIZE 1

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22



void setup() 
{
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Serial port for debugging purposes
  Serial.begin(115200);
  //DEPRECATED: system_get_chip_id(); NOT AVAILABLE IN ESP32 SDK?
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while ((WiFi.status() != WL_CONNECTED) and (connectCount < 10)) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    connectCount = connectCount + 1;
  }
  
  if (!SPIFFS.begin(true)) 
  {
    Serial.println("An Error has occurred while mounting SPIFFS.");
    ESP.restart();
  }
  else 
  {
    delay(500);
    Serial.println("SPIFFS mounted successfully.");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    // Print ESP32 Local IP Address
    Serial.print("IP Address: http://");
    Serial.println(WiFi.localIP());
    Serial.println();
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println();
    Serial.print("APMAC: ");
    Serial.println(WiFi.softAPmacAddress());
  }
  
  // Set LED pins to output.
  pinMode(33, OUTPUT); // On-board red LED, TODO: flash as a status indicator.
  pinMode(4, OUTPUT);  // On-board flash light LED, also used by SD card (HS2_DATA1).
  pinMode(13, INPUT);  // PIR signal input, active low because of 47k ohm pullup resistor.
  
  digitalWrite(33, LOW);
  digitalWrite(4, LOW);
  
  // TODO: Test the broken out GPIO pins: 0,1,2,3,4 and 12,13,14,15,16 for the PIR signal line.
  // GPIO0: camera CSI_MCLK, has a 10k pullup resistor connected to 3V3 rail.
  // GPIO1: U0TXD
  // GPIO2: SD card
  // GPIO3: U0RXD
  // GPIO4: SD card
  // All the SD card data lines have 10k pullup resistors on them, except GPIO12, 
  // says NC on circuit diagram but who knows?
  // TODO: power up and test voltages at the pin headers.
  //pinMode(12, INPUT);  used by SD card
  //pinMode(13, INPUT);  used by SD card
  //pinMode(14, INPUT);  used by SD card
  //pinMode(16, INPUT);  Used for UART2 U2RXD and PSRAM CS# pin, has a 10k pullup resistor connected to 3V3 rail.

  // OV2640 camera module
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) 
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } 
  else 
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) 
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

/*
  Serial.println("Starting SD Card...");
  if(!SD_MMC.begin())
  {
    Serial.println("SD Card Mount Failed.");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached.");
    return;
  }
  Serial.printf("SD Card attached type: %i\n", cardType);

*/

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  //EEPROM.write(0, 0);
  //EEPROM.commit();

  if (WiFi.status() == WL_CONNECTED)
  {
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html);
    });
  
    server.on("/capture", HTTP_GET, [](AsyncWebServerRequest * request) {
      takeNewPhoto = true;
      request->send_P(200, "text/plain", "Taking Photo");
    });
  
    server.on("/saved-photo", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, FILE_PHOTO, "image/jpg", false);
    });
  
    server.on("/switch", HTTP_GET, [](AsyncWebServerRequest * request) {
      switchLEDS = true;
      request->send_P(200, "text/plain", "Switching LEDS");
    });
    
    // Start server
    server.begin();
  }
}

void loop() 
{
  if (takeNewPhoto) 
  {
    digitalWrite(4, HIGH); // Turn on the flash LED.
    capturePhotoSave();
    digitalWrite(4, LOW);  // Turn off the flash LED.
    takeNewPhoto = false;
  }
  if (switchLEDS)
  {
    flipled();
    switchLEDS = false;
  }
  if (delayCount > 100) // Ignore signal for 10 seconds, the pir signal is pretty dodgy.
  {
    delayCount = 0;
    if (digitalRead(13) == LOW) // PIR input signal, active low.
    {
      Serial.println("PIR motion detected...");
      takeNewPhoto = true;
      flipled();
    }
  }
  else
  {
    delayCount = delayCount + 1;
  }
  
  //TODO: function to blink led without delay()
  delay(100);
}

void flipled()
{
    Serial.println("Switching LEDS...");
    if (ledsOn)
    {
       //digitalWrite(4, LOW);
       digitalWrite(33, HIGH);     
    }
    else
    {
       //digitalWrite(4, HIGH);
       digitalWrite(33, LOW); 
    }
    ledsOn = !ledsOn;
  
}
// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) 
{
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  f_pic.close();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSave( void ) 
{
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly

  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    fb = esp_camera_fb_get();
    if (!fb) 
    {
      Serial.println("Camera capture failed");
      return;
    }
    else
    {
      Serial.printf("Got image size: %i\n", fb->len);
    }

    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File spiffsfile = SPIFFS.open(FILE_PHOTO, FILE_WRITE);

    // Insert the data in the photo file
    if (!spiffsfile) 
    {
      Serial.println("Failed to open file in writing mode");
    }
    else 
    {
      int fsize = 0;
      while (fsize == 0)
      {
         spiffsfile.write(fb->buf, fb->len); // payload (image), payload length
         spiffsfile.flush();
         Serial.print("The picture has been saved in ");
         Serial.print(FILE_PHOTO);
         Serial.print(" - Size: ");
         Serial.print(spiffsfile.size());
         Serial.println(" bytes");
         fsize = spiffsfile.size();
      }
    }
    // Close the file
    spiffsfile.close();

    pictureNumber = EEPROM.read(0) + 1;
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
    
    // Now save the picture to the DS card and update the picture number in flash memory.
    
    // Path where new picture will be saved in SD Card
    
    
    /*
    String path = "/camxxx" + String(pictureNumber) +".jpg";

    fs::FS &fs = SD_MMC; 
    Serial.printf("Picture file name: %s\n", path.c_str());
  
    File sdfile = fs.open(path.c_str(), FILE_WRITE);
    if(!sdfile)
    {
      Serial.println("Failed to open SD file in writing mode.");
    } 
    else 
    {
      sdfile.write(fb->buf, fb->len); // payload (image), payload length
      Serial.printf("Saved SD file to path: %s\n", path.c_str());
      EEPROM.write(0, pictureNumber);
      EEPROM.commit();
    }
    sdfile.close();
    */

    
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  } while ( !ok );
}
