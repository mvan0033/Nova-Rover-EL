/*
Leigh Oliver 07/05/2021
This script will test the readings of whatever ADC chips it finds (of the four total).
It will also sweep the PWM output of the PWM driver, on all channels.
*/
#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include "Adafruit_TLC59711.h"

// IMPORTANT BITS
#include <adc_nonblocking.h>
ADCHandler adcCurrent = ADCHandler();
// TO USE
// Setup
// adc.init(<i2caddr>,<average sample count>)
// Loop
// adc.update() (ALWAYS call this!)
// adc.readAverage(<channel number>)
// adc.readLatest(<channel number>)
// Enjoy

// // ADCs (0x68,0x69,0x6B,0x6C)
// MCP342x adc1 = MCP342x(0x6B);
// MCP342x adc2 = MCP342x(0x68); 
// MCP342x adc3 = MCP342x(0x6C);  

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  adcCurrent.init(0x68,4);      
}

unsigned long dt = millis();

void loop(void)
{
  // Update adc
  adcCurrent.update();

  if(millis() - dt > 250)
  {
    Serial.println("CH1,  CH2,  CH3,  CH4");
    Serial.print(adcCurrent.readLatest(1));
    Serial.print(",");
    Serial.print(adcCurrent.readLatest(2));
    Serial.print(",");
    Serial.print(adcCurrent.readLatest(3));
    Serial.print(",");
    Serial.println(adcCurrent.readLatest(4));
    

    Serial.println("AV1,  AV2,  AV3,  AV4");
    Serial.print(adcCurrent.readAverage(1));
    Serial.print(",");
    Serial.print(adcCurrent.readAverage(2));
    Serial.print(",");
    Serial.print(adcCurrent.readAverage(3));
    Serial.print(",");
    Serial.println(adcCurrent.readAverage(4));
    dt = millis();
  }
}