/*
Leigh Oliver 07/05/2021
This script will test the readings of whatever ADC chips it finds (of the four total).
It will also sweep the PWM output of the PWM driver, on all channels.
*/
#include <Arduino.h>
#include <Wire.h>
#include <MCP342x.h>
#include <SPI.h>
#include "Adafruit_TLC59711.h"
#include <pwm_utils.h>
#include <adc_utils.h>
#include <PID_v1.h>
#include <math.h>

// Setup PWM for testing
#define NUM_TLC59711 1

#define data   8
#define clock  7

Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);

// IMPORTANT BITS
#include <adc_nonblocking.h>
ADCHandler adcCurrent = ADCHandler();

// TO USE
// Setup
// adc.init(<i2caddr>,<average sample count>)
// Loop
// adc.update() (ALWAYS call this!)
// adc.readAverage(<channel number>)
// adc.readLatest(<channel number>)
// Enjoy

// // ADCs (0x68,0x69,0x6B,0x6C)
// MCP342x adc1 = MCP342x(0x6B);
// MCP342x adc2 = MCP342x(0x68); 
// MCP342x adc3 = MCP342x(0x6C);  

// PID system for tracking to a desired voltage

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=10000, Ki=0.0, Kd=25.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
unsigned long RESOLUTION = 18;
unsigned long ADCMAX = 0;

void setup(void)
{
  // Setup serial
  Serial.begin(115200);
  Wire.begin();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,65535);

  tlc.begin();
  tlc.write();

  ADCMAX = pow(2,RESOLUTION-1)-1;
  Serial.print("ADCMAX: ");
  Serial.println(ADCMAX);

  Serial.print("kP: ");
  Serial.println(Kp);
  Serial.print("kI: ");
  Serial.println(Ki);
  Serial.print("kD: ");
  Serial.println(Kd);
  
  Serial.println("MILLIS,DUTY,TARGET,RAW,AVERAGE");

  pwm_set_duty_all(&tlc,0);

  adcCurrent.init(0x68,32);      
}


uint16_t duty = 0; // Out of 65535
int stat = 0;


void loop(void)
{ 
  // 3 Volts
  Setpoint = 3.0;

  // Update adc
  stat = adcCurrent.update();

  if(stat == 3)
  {
    // Update the pid input
    Input = raw_to_voltage(adcCurrent.readLatest(3),5.0,ADCMAX);
    
    // Compare Input vs Setpoint, to ajust PID constants
    if(Setpoint - Input > 0.25)
    {
      Kp=10000;
      Ki=0.0;
      Kd=25.0;
      myPID.SetTunings(Kp,Ki,Kd);
    }else if(Setpoint - Input > 0.025){
      Kp=5000;
      Ki=0.0;
      Kd=25.0;
      myPID.SetTunings(Kp,Ki,Kd);
    }else if(Setpoint - Input > 0.0025){
      Kp=1000;
      Ki=0.0;
      Kd=25.0;
      myPID.SetTunings(Kp,Ki,Kd);
    }else{
      Kp=1000;
      Ki=0.0;
      Kd=5.0;
      myPID.SetTunings(Kp,Ki,Kd);
    }

    myPID.Compute();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(duty);
    Serial.print(",");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(raw_to_voltage(adcCurrent.readLatest(3),5.0,ADCMAX),6);
    Serial.print(",");
    Serial.println(raw_to_voltage(adcCurrent.readAverage(3),5.0,ADCMAX),6);

    // Apply output
    duty += (int16_t)Output;
    pwm_set_duty_all(&tlc,duty);
  }
}