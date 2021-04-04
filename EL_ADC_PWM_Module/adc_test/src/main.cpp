#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include <utils.h>
#include <adc_utils.h>

/* Demonstrate the use of convertAndRead().
 */

// 0x68 is the default address for all MCP342x devices
// 0x68
// 0x69
// 0x6B
// 0x6C

// 0x6B is current (shoud be 0x69)
// 0x69 is temp (should be 0x6B)

uint8_t adc_temp_addr = 0x69;
uint8_t adc_current_addr = 0x6B;

MCP342x adc_temp = MCP342x(adc_temp_addr);
MCP342x adc_current = MCP342x(adc_current_addr);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  if(!util_check_i2c_device_exists(adc_current_addr))
  {
    Serial.print("No device found at ");
    Serial.println(adc_current_addr,HEX);
    while(1){
      ;;
    }
  }

  if(!util_check_i2c_device_exists(adc_temp_addr))
  {
    Serial.print("No device found at ");
    Serial.println(adc_current_addr,HEX);
    while(1){
      ;;
    }
  }
}

void loop(void)
{
  // Read using convenience functions
  double value_current_voltage = adc_read_voltage(&adc_current,MCP342x::channel1,5);
  Serial.print("CURRENT V: ");
  Serial.println(value_current_voltage,6);
  Serial.print("CURRENT I(A): ");
  Serial.println(util_voltage_to_current(value_current_voltage));
  Serial.println("---");

  double value_temp_voltage = adc_read_voltage(&adc_temp,MCP342x::channel1,5);
  Serial.print("TEMP V: ");
  Serial.println(value_temp_voltage);
  Serial.print("TEMP Cdeg: ");
  Serial.println(util_voltage_to_temperature(value_temp_voltage));
  Serial.println("---");  


  delay(1000);
}