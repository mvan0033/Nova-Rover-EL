/*
This script will:
- Increment the PWM output to the MOSFET gate driver, while:
- Measuring a voltage on CH1 of the Current ADC (Channel 0x6B)
- And then calculating the equivalent resistance of the MOSFET, as it runs through a voltage divider.
- I used a Top resistor of 1Meg, with the MOSFET acting as the bottom resistor.

We will then be able to obtain a nice PWM vs MOSFET resistance curve, at a V_DS of 0V.
*/
#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include <utils.h>
#include <adc_utils.h>



#include "Adafruit_TLC59711.h"
#include <SPI.h>
// PWM
#define NUM_TLC59711 1
#define data   8
#define clock  7
uint8_t MOSFET_PWM_CH = 6;
Adafruit_TLC59711 pwmModule = Adafruit_TLC59711(NUM_TLC59711, clock, data);




// ADCs (0x68,0x69,0x6B,0x6C)
uint8_t adc_current_addr = 0x69;
uint8_t adc_temp_addr = 0x6B;
MCP342x adc_temperature = MCP342x(adc_temp_addr);
MCP342x adc_current = MCP342x(adc_current_addr);

// Global variables.
double current_voltageRef = 2.468776;

void set_mosfet_pwm(uint16_t pwmValue);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  // Setup PWM
  Serial.print("Setting up PWM module...");
  pwmModule.begin();
  pwmModule.write();
  pwmModule.simpleSetBrightness(127);
  pwmModule.write();
  set_mosfet_pwm(0);
  Serial.println("Done.");

  // Reset devices
  Serial.print("Calling reset of all MCP3424 chips...");
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  Serial.println("Done.");

  if(!util_check_i2c_device_exists(adc_current_addr))
  {
    Serial.print("No device found at ");
    Serial.println(adc_current_addr,HEX);
    Serial.println("Could not find ADC for Current Measurement.");
    while(1){
      ;;
    }
  }

  if(!util_check_i2c_device_exists(adc_temp_addr))
  {
    Serial.print("No device found at ");
    Serial.println(adc_current_addr,HEX);
    Serial.println("Could not find ADC for Temperature Measurement.");
    while(1){
      ;;
    }
  }

  // Calibrate the zero voltage reference
  current_voltageRef = 0;
  Serial.println("Calibrating CURRENT SENSOR voltage reference.");

  for(int i =0; i<25; i++)  
  {
    double temp = adc_read_voltage(&adc_current,MCP342x::channel1,5);
    current_voltageRef += temp/25;
    Serial.print(i);
    Serial.println("/25");
  }
  Serial.print("CURRENT SENSOR VREF: ");
  Serial.println(current_voltageRef,6);

}

void set_mosfet_pwm(uint16_t pwmValue)
{
  // SET PWM.
  uint16_t pwm = 65535 - pwmValue;
  
  if(pwm < 0)
  {
    pwm = 0;
  }
  if(pwm > 65535)
  {
    pwm = 65535;
  }

  pwmModule.setPWM(MOSFET_PWM_CH,pwm);
  pwmModule.write();
  // Wait.
  delay(10);
}

uint16_t pwmValue = 0000;

void loop(void)
{
  set_mosfet_pwm(pwmValue);
  Serial.print("PWM VALUE: ");
  Serial.println(pwmValue);
  // Serial.print(",");
  // Serial.println("---");

  // Read using convenience functions
  double value_mosfet_voltage = adc_read_voltage(&adc_current,MCP342x::channel1,5);
  // Serial.print("CURRENT V: ");
  // Serial.println(value_mosfet_voltage,6);
  Serial.print("MOSFET Current(A): ");
  Serial.println(util_voltage_to_current(value_mosfet_voltage,current_voltageRef),6);
  // Serial.print(",");

  // Also monitor temperature for fun.
  double value_temp_voltage = adc_read_voltage(&adc_temperature,MCP342x::channel1,5);
  // Serial.print("TEMP V: ");
  // Serial.println(value_temp_voltage);
  Serial.print("TEMP Cdeg: ");
  Serial.print(util_voltage_to_temperature(value_temp_voltage),1);
  Serial.println("");
  Serial.println("");  

  pwmValue += 100;
  if(pwmValue > 65535)
  {
    pwmValue = 0;
  }

  delay(1000);

}