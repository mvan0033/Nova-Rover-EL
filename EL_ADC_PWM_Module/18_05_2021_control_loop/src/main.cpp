/*
Leigh Oliver 18/05/2021
Control feedback loop to maintain a set current.
*/
#include <Arduino.h>
#include <control_loop.h>

// Control loop object which handles our PWM/ADC hardware.
ControlLoop controller;

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  // Setup control loop object (which initialises ADC/PWM hardware)
  controller.init();
}

void loop(void)
{
  // Set the controller mode
  controller.set_target_mode(0); // Current mode
  
  // Set the controller target
  controller.set_target_value(10);

  // Set the controller output
  controller.set_enable(true);
  
  // Update control loop
  controller.update();

  // // Sweep the PWM
  // pwmOutputValue ++;
  // if(pwmOutputValue >= PWM_MAX)
  // {a
  //   pwmOutputValue = 0;
  // }
  
  // pwm_set_duty_all(&pwm_module,pwmOutputValue);
  // Serial.print("PWM DUTY: ");
  // Serial.println(pwmOutputValue);

  // // Read ADC channels
  // for(int i = 0; i<4; i++)
  // {
  //   // Check if valid flag is 1 for this address
  //   if(adc_addrs_found[i] == 1)
  //   {
  //     Serial.print("ADC 0x");
  //     Serial.print(adc_addrs[i],HEX);
  //     Serial.print(",CH1: ");
  //     Serial.print(adc_read_raw(&adc_module[i],MCP342x::channel1));
  //     Serial.print(",CH2: ");
  //     Serial.print(adc_read_raw(&adc_module[i],MCP342x::channel2));
  //     Serial.print(",CH3: ");
  //     Serial.print(adc_read_raw(&adc_module[i],MCP342x::channel3));
  //     Serial.print(",CH4: ");
  //     Serial.println(adc_read_raw(&adc_module[i],MCP342x::channel4));
  //   }
  // }
}