/**
 * Project: mosfet-test.ino
 *
 * Description: Test mosfet gate control to determine optimal switching circuit at various switching frequencies.
 *
 * Author: Derek Chadwick
 * Date: 07/04/2021
 */

#include <Arduino.h>

#define RS232 Serial




uint8_t blink;

void setup() {
  // Turn-off the 'brownout detector'
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  
    RS232.begin(115200);
    
    RS232.println();
    RS232.println();
    RS232.println();
    RS232.println();
    RS232.println();
    
    pinMode(2, OUTPUT);      // set the on-board blue LED pin mode
    pinMode(25, OUTPUT);     // MOSFET gate.
    blink = 0;
    
    for(uint8_t t = 4; t > 0; t--) 
    {
        RS232.printf("Flushing the pipes %d...\n", t);
        RS232.flush();
        delay(1000);
    }

}

void loop() {

    int ii;

    for(ii = 0; ii < 5; ii++)
    {
       digitalWrite(25, HIGH);
       digitalWrite(2, HIGH);
       delay(1000);
       digitalWrite(25, LOW);
       digitalWrite(2, LOW);
       delay(1000);
    }
    for(ii = 0; ii < 10; ii++)
    {
       digitalWrite(25, HIGH);
       digitalWrite(2, HIGH);
       delay(500);
       digitalWrite(25, LOW);
       digitalWrite(2, LOW);
       delay(500);
    }    
    for(ii = 0; ii < 20; ii++)
    {
       digitalWrite(25, HIGH);
       digitalWrite(2, HIGH);
       delay(250);
       digitalWrite(25, LOW);
       digitalWrite(2, LOW);
       delay(250);
    }
    for(ii = 0; ii < 40; ii++)
    {
       digitalWrite(25, HIGH);
       digitalWrite(2, HIGH);
       delay(125);
       digitalWrite(25, LOW);
       digitalWrite(2, LOW);
       delay(125);
    }
    for(ii = 0; ii < 80; ii++)
    {
       digitalWrite(25, HIGH);
       digitalWrite(2, HIGH);
       delay(50);
       digitalWrite(25, LOW);
       digitalWrite(2, LOW);
       delay(50);
    }    
    for(ii = 0; ii < 160; ii++)
    {
       digitalWrite(25, HIGH);
       digitalWrite(2, HIGH);
       delay(10);
       digitalWrite(25, LOW);
       digitalWrite(2, LOW);
       delay(10);
    }    
    RS232.printf("End of test loop...\n");

    delay(1000);
}
