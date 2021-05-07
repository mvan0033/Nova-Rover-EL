#include "Adafruit_TLC59711.h"

void pwm_set_duty(Adafruit_TLC59711 *pwm_obj, uint8_t channel, uint16_t pwmValue)
{
  /*
  Sets the PWM in a standard way, where 0 is 0 duty, and PWM_MAX is 100% duty.
  */

  // Flip PWM reading for the module (as it's a pull-up output)
  uint16_t pwm = 65535 - pwmValue;
  
  // Clamp PWM 
  if(pwm < 0)
  {
    pwm = 0;
  }

  if(pwm > 65535)
  {
    pwm = 65535;
  }

  // Write to the module
  pwm_obj->setPWM(channel,pwm);
  pwm_obj->write();

  // Wait for data to be sent (safety)
  delay(10);
}

void pwm_set_duty_all(Adafruit_TLC59711 *pwm_obj, uint16_t pwmValue)
{
  for(int i = 0; i<12; i++)
  {
    pwm_set_duty(pwm_obj,i,pwmValue);
  }
}