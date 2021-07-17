/*
Leigh Oliver 18/05/2021
Control feedback loop to maintain a set current.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <control_loop.h>

// Define ADC function by addrress
uint8_t adc_temperature_addr = 0x6B; // MARKED AS 0X69 ON THE PCB
uint8_t adc_current_addr = 0x68; // MARKED AS 0X68 ON THE PCB
uint8_t adc_voltage_addr = 0x6C; // MARKED AS 0X6A ON THE PCB

// Setup the PWM stuff
Adafruit_TLC59711 pwmModule = Adafruit_TLC59711(1,7,8);
ADCHandler adc_temperature = ADCHandler(adc_temperature_addr);
ADCHandler adc_current = ADCHandler(adc_current_addr);
ADCHandler adc_voltage = ADCHandler(adc_voltage_addr);

// Control loop object which handles our PWM/ADC hardware.
ControlLoop controller = ControlLoop(&pwmModule,&adc_temperature,&adc_current,&adc_voltage);

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  // Setup control loop object
  // Attach out modules
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

  // Check error state (true means error)
  if(controller.get_error_state())
  {
    Serial.println("Resetting...");
    delay(5000);
    controller.reset_errors();
  }
}