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
MCP342x adc = MCP342x(0x68); // (Current ADC)

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

int channel = 1;
long lastResult = 0;

void loop(void)
{
  MCP342x::Config configIn;
  long result;
  MCP342x::Config configOut;

  switch(channel)
  {
    case 1:    
      configIn = MCP342x::Config(MCP342x::channel1,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);
      break;
    case 2:
      configIn = MCP342x::Config(MCP342x::channel2,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);
      break;
    case 3:
      configIn = MCP342x::Config(MCP342x::channel3,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);
      break;
    case 4:
      configIn = MCP342x::Config(MCP342x::channel4,MCP342x::continous,MCP342x::resolution18,MCP342x::gain1);
      break;
  }

  adc.configure(configIn);
  adc.read(result,configOut);

  // Only prints when ready!
  if(configOut.isReady())
  {
    Serial.print("CH");
    Serial.print(channel);
    Serial.print(": ");
    Serial.println(result);
    channel++;
    if(channel == 5)
    {
      channel = 1;
    }
  }else{
    Serial.print(".");
  }
  
  // if(result == lastResult)
  // {
  //   channel++;
  //   if(channel == 5)
  //   {
  //     channel = 1;
  //   }
  //   lastResult = 0;
  // }
  // lastResult = result;
}