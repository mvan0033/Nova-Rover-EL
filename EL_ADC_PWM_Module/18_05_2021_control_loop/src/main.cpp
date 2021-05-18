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
  controller.set_target_value(10); // 10 Amp

  // Set the controller output
  controller.set_enable(true);
  
  // Update control loop!
  controller.update();

  // Check error state (true means error)
  if(controller.get_error_state())
  {
    Serial.println("Resetting...");
    delay(5000);
    controller.reset_errors();
  }
}