/*
Leigh Oliver 18/05/2021
Control feedback loop to maintain a set current.
*/
#include <Arduino.h>
#include <control_loop.h>

// Control loop object which handles our PWM/ADC hardware.
ControlLoop controller;

double averageValueList[128];
int avgIndex = 0;
bool avgFull = false;
double averageVal = 0;

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  // Setup control loop object (which initialises ADC/PWM hardware)
  controller.init();

  // Set the controller mode
  controller.set_target_mode(0); // Current mode
  
  // Set the controller target
  controller.set_target_value(10); // 25 Amps

  // Set the controller output
  controller.set_enable(true);
}

void loop(void)
{
  // Update control loop!
  controller.update();
  
  averageValueList[avgIndex++] = controller.get_total_current();
  if(avgIndex >= 128)
  {
    avgIndex = 0;
    avgFull = true;
  }

  int avgLen = 128;
  if(!avgFull)
  {
    avgLen = avgIndex;
  }
  
  averageVal = 0;

  for(int i = 0; i<avgLen; i++)
  {
    averageVal += averageValueList[i]/(double)avgLen;
  }


  Serial.print("CURRENT AVG: ");
  Serial.println(averageVal,6);

  // Check error state (true means error)
  if(controller.get_error_state())
  {
    Serial.println("Resetting...");
    delay(5000);
    controller.reset_errors();
  }
}