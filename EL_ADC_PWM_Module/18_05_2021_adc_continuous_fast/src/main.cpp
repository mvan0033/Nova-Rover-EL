/*
Leigh Oliver 07/05/2021
This script will test the readings of whatever ADC chips it finds (of the four total).
It will also sweep the PWM output of the PWM driver, on all channels.
*/
#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include "Adafruit_TLC59711.h"

#include <utils.h>
#include <adc_utils.h>

// ADCs (0x68,0x69,0x6B,0x6C)
MCP342x adc1 = MCP342x(0x6B);
MCP342x adc2 = MCP342x(0x68); 
MCP342x adc3 = MCP342x(0x6C);  

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  // Reset devices
  Serial.print("Calling reset of all MCP3424 chips...");
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  Serial.println("Done.");

  if(!util_check_i2c_device_exists(0x68))
  {
    Serial.println("ADC NOT FOUND.");
  }      
}

int channel1 = 1;
int channel2 = 1;
int channel3 = 1;

unsigned long adc1channel1Timer = millis();

MCP342x::Resolution res = MCP342x::resolution14;

void loop(void)
{
  long result1;
  long result2;
  long result3;
  MCP342x::Config configIn1;
  MCP342x::Config configIn2;
  MCP342x::Config configIn3;
  MCP342x::Config configOut1;
  MCP342x::Config configOut2;
  MCP342x::Config configOut3;

  switch(channel1)
  {
    case 1:    
      configIn1 = MCP342x::Config(MCP342x::channel1,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 2:
      configIn1 = MCP342x::Config(MCP342x::channel2,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 3:
      configIn1 = MCP342x::Config(MCP342x::channel3,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 4:
      configIn1 = MCP342x::Config(MCP342x::channel4,MCP342x::continous,res,MCP342x::gain1);
      break;
  }

  switch(channel2)
  {
    case 1:    
      configIn2 = MCP342x::Config(MCP342x::channel1,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 2:
      configIn2 = MCP342x::Config(MCP342x::channel2,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 3:
      configIn2 = MCP342x::Config(MCP342x::channel3,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 4:
      configIn2 = MCP342x::Config(MCP342x::channel4,MCP342x::continous,res,MCP342x::gain1);
      break;
  }

  switch(channel3)
  {
    case 1:    
      configIn3 = MCP342x::Config(MCP342x::channel1,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 2:
      configIn3 = MCP342x::Config(MCP342x::channel2,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 3:
      configIn3 = MCP342x::Config(MCP342x::channel3,MCP342x::continous,res,MCP342x::gain1);
      break;
    case 4:
      configIn3 = MCP342x::Config(MCP342x::channel4,MCP342x::continous,res,MCP342x::gain1);
      break;
  }

  // Setup ADCs
  adc1.configure(configIn1);
  adc2.configure(configIn2);
  adc3.configure(configIn3);

  adc1.read(result1,configOut1);
  adc2.read(result2,configOut2);
  adc3.read(result3,configOut3);

  // Only prints when ready!
  if(configOut1.isReady())
  {
    Serial.print("ADC1 CH");
    Serial.print(channel1);
    Serial.print(": ");
    Serial.println(result1);

    if(channel1 == 1)
    {
      unsigned long deltaTime = millis() - adc1channel1Timer;
      Serial.print("CH1 Sample Time: ");
      Serial.println(deltaTime);
      adc1channel1Timer = millis();
    }

    channel1++;
    if(channel1 == 5)
    {
      channel1 = 1;
    }
  }

  // Only prints when ready!
  if(configOut2.isReady())
  {
    Serial.print("ADC2 CH");
    Serial.print(channel2);
    Serial.print(": ");
    Serial.println(result2);
    channel2++;
    if(channel2 == 5)
    {
      channel2 = 1;
    }
  }

  // Only prints when ready!
  if(configOut3.isReady())
  {
    Serial.print("ADC3 CH");
    Serial.print(channel3);
    Serial.print(": ");
    Serial.println(result3);
    channel3++;
    if(channel3 == 5)
    {
      channel3 = 1;
    }
  }
}