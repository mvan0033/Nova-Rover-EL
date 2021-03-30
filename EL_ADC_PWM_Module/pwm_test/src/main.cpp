#include <Arduino.h>
#include "Adafruit_TLC59711.h"
#include <SPI.h>

// How many boards do you have chained?
#define NUM_TLC59711 1

#define data   8
#define clock  7

Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);
//Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711);

void setup() {
  Serial.begin(9600);
  
  Serial.println("TLC59711 test");
  
  tlc.begin();
  tlc.write();
  tlc.simpleSetBrightness(127);
  tlc.write();
}

void loop() {
  // Set led
  for(int i = 0; i<65536; i+=10)
  {
    tlc.setPWM(0,65535);
    tlc.write();
    delay(10);
  }
}