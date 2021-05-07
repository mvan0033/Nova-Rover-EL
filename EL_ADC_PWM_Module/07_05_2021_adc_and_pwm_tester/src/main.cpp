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
#include <pwm_utils.h>
#include <adc_utils.h>
#include <SPI.h>

// PWM
#define NUM_TLC59711 1
#define data   8
#define clock  7
#define PWM_MAX 65535
Adafruit_TLC59711 pwm_module = Adafruit_TLC59711(NUM_TLC59711, clock, data);

// ADCs (0x68,0x69,0x6B,0x6C)
uint8_t adc_addrs[4] = {0x68,0x69,0x6B,0x6C};
bool adc_addrs_found[4] = {0,0,0,0}; // Flags indicating found addresses
MCP342x adc_module[4] = {MCP342x(adc_addrs[0]),MCP342x(adc_addrs[1]),MCP342x(adc_addrs[2]),MCP342x(adc_addrs[3])};

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  // Setup PWM module
  Serial.print("Setting up PWM module...");
  pwm_module.begin();
  pwm_module.write();
  pwm_module.simpleSetBrightness(127);
  pwm_module.write();
  pwm_set_duty_all(&pwm_module,0);
  Serial.println("Done.");

  // Reset devices
  Serial.print("Calling reset of all MCP3424 chips...");
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  Serial.println("Done.");

  // Check for ADC devices
  for(int i = 0; i<4; i++)
  {
    Serial.print("Polling I2C for address 0x");
    Serial.print(adc_addrs[i],HEX);
    Serial.print("...");
    if(!util_check_i2c_device_exists(adc_addrs[i]))
    {
      Serial.println("NOT FOUND.");
      adc_addrs_found[i] = 0;
    }else{
      Serial.println("FOUND");
      adc_addrs_found[i] = 1;
    }
  }
}

uint16_t pwmOutputValue = 0;

void loop(void)
{
  // Sweep the PWM
  pwmOutputValue ++;
  if(pwmOutputValue >= PWM_MAX)
  {
    pwmOutputValue = 0;
  }
  
  pwm_set_duty_all(&pwm_module,pwmOutputValue);
  Serial.print("PWM DUTY: ");
  Serial.println(pwmOutputValue);

  // Read ADC channels
  for(int i = 0; i<4; i++)
  {
    // Check if valid flag is 1 for this address
    if(adc_addrs_found[i] == 1)
    {
      Serial.print("ADC 0x");
      Serial.print(adc_addrs[i],HEX);
      Serial.print(",CH1: ");
      Serial.print(adc_read_raw(&adc_module[i],MCP342x::channel1));
      Serial.print(",CH2: ");
      Serial.print(adc_read_raw(&adc_module[i],MCP342x::channel2));
      Serial.print(",CH3: ");
      Serial.print(adc_read_raw(&adc_module[i],MCP342x::channel3));
      Serial.print(",CH4: ");
      Serial.println(adc_read_raw(&adc_module[i],MCP342x::channel4));
    }
  }
}