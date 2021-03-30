#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include "Adafruit_TLC59711.h"
#include <SPI.h>

// How many boards do you have chained?
#define NUM_TLC59711 1

#define data   8
#define clock  7

Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);

/* Demonstrate the use of convertAndRead().
 */

// 0x68 is the default address for all MCP342x devices
// 0x68
// 0x69
// 0x6B
// 0x6C
uint8_t address = 0x68;
MCP342x adc = MCP342x(address);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  // Setup PWM
  tlc.begin();
  tlc.write();
  tlc.simpleSetBrightness(127);
  tlc.write();

  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  // Check device present
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available())
  {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1)
      ;
  }
}

int i = 0;

void loop(void)
{
  // SET PWM
  if(i < 65536-10)
  {
    i+= 10;
  }else{
    i = 0;
  }
  tlc.setPWM(0,i);
  tlc.write();
  delay(10);

  // READ BACK
  long value = 0;

  MCP342x::Config status; // Return value
  
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution18, MCP342x::gain1,1000000, value, status);

  if (err)
  {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else
  {
    Serial.print("Value: ");
    Serial.println(value);
  }

  // delay(1000);
}