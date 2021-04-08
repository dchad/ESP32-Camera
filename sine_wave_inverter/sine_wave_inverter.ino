/*
 * Project: ESP32 DC - AC Inverter
 * 
 * Description: Implements the 50Hz AC sine wave driver module for a DC to AC inverter.
 *              The power module consists of a power mosfet push-pull circuit using
 *              a transformer with a center tapped 12V/24V primary winding and 240V secondary winding.
 *              The ESP32 uses DAC0 and DAC1 to generate the split sine wave pulses to drive each side
 *              of the mosfet push-pull circuit.
 * 
 * 
 * Author: Derek Chadwick
 * 
 * Date: 24/05/2020
 */


#include "Arduino.h"
#include "time.h"

#define write_interval 39 // Number of microseconds between DAC writes for a 50Hz half sine wave using 180 output values between 0 to 255
                          // equals 10000 usecs divided by 180: 10000 / 180 = 55.555555555 usecs, then lower to 39 usecs to compensate for processing time.


//  for (int deg = 0; deg < 360; deg = deg + 8){
//    dacWrite(25, int(128 + 80 * (sin(deg*PI/180)+sin(3*deg*PI/180)/3+sin(5*deg*PI/180)/5+sin(7*deg*PI/180)/7+sin(9*deg*PI/180)/9+sin(11*deg*PI/180)/11))); // Square
// ESP32 has two 8-bit DAC (digital to analog converter) channels, connected to GPIO25 (Channel 1) and GPIO26 (Channel 2)
// Square wave   = amplitude . sin(x) + sin(3.x) / 3 +  sin (5.x) / 5 + sin (7.x) / 7  + sin (9.x) / 9  + sin (11.x) / 11  Odd harmonics
// Triangle wave = amplitude . sin(x) - 1/3^2.sin(3.x) +  1/5^2.sin(5.x) - 1/7^2.sin (7.x) + 1/9^2.sin(9.x) - 1/11^2.sin (11.x) Odd harmonics
// dacWrite(25, int(128 + 80 * (sin(deg*PI/180)))); // GPIO Pin mode (OUTPUT) is set by the dacWrite function
// dacWrite(25, int(128 + 80 * (sin(deg*PI/180)+sin(3*deg*PI/180)/3+sin(5*deg*PI/180)/5+sin(7*deg*PI/180)/7+sin(9*deg*PI/180)/9+sin(11*deg*PI/180)/11))); // Square
// dacWrite(25, int(128 + 80 * (sin(deg*PI/180)+1/pow(3,2)*sin(3*deg*PI/180)+1/pow(5,2)*sin(5*deg*PI/180)+1/pow(7,2)*sin(7*deg*PI/180)+1/pow(9,2)*sin(9*deg*PI/180)))); // Triangle

void setup() 
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);      // Devkit 1.0 has a blue LED on pin 2, set the LED pin mode to output.
  delay(10);
  digitalWrite(2, HIGH);   // turn the LED on.
  Serial.println("Setup finished...");
  int out_val;
  int ii;

  for (ii = 0; ii < 181; ii++)
  {
    out_val = int(256 * sin(ii *  0.017453)); // Convert degrees to radians and multiply by the amplitude, PI / 180 = 0.017453
    Serial.println(out_val);
  }  
}


void loop() 
{
  // Positive cycle pulse generation DAC0
  int ii;
  for (ii = 0; ii < 180; ii++)
  {
    dacWrite(25, int(256 * sin(ii *  0.017453))); // Convert degrees to radians and multiply by the amplitude, PI / 180 = 0.017453
    delayMicroseconds(write_interval);
  }
  // Negative half of pulse generation DAC0
  //for (ii = 255; ii >= 0; ii--)
  //{
  //  dacWrite(25, ii);
  //  delayMicroseconds(20);
  //}
  // Negative pulse generation DAC1
  for (ii = 0; ii < 180; ii++)
  {
    dacWrite(26, int(256 * sin(ii * 0.017453)));
    delayMicroseconds(write_interval);
  }
  // Negative pulse generation DAC1
  //for (ii = 255; ii >= 0; ii--)
  //{
  //  dacWrite(26, ii);
  //  delayMicroseconds(20);
  //}
}
