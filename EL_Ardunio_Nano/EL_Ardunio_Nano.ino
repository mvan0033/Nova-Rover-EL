// Importing
#include "LiquidCrystal_I2C.h"
#include <Wire.h>

// Global variables
#define rotary_encoder_clk 2
#define rotary_encoder_dt 3
#define rotary_encoder_sw 4
volatile bool clockwise;
volatile bool turnDetected = false;
volatile bool buttonPressed = false;
float power = 0;
float current = 0;
float curCurrent = 0;
float curPower = 0;
float lowVolt = 0;
float hotTemp = 0;
int elapsedTime = 0;
int lcdAddress = 0x27;
int screen = 1;
volatile bool powerMode = false;
int cursorPosition = 0;


LiquidCrystal_I2C lcd(lcdAddress, 128, 64);

// Custom LCD Characters
//byte rightArrow[8] = 
//{
//  0b10000,
//  0b11000,
//  0b11100,
//  0b11110,
//  0b11110,
//  0b11100,
//  0b11000,
//  0b10000
//};

void isr0()
{
  turnDetected = true;
  clockwise = digitalRead(rotary_encoder_clk) != digitalRead(rotary_encoder_dt);
    Serial.print("\nTurn detected!\n");
//  Serial.print(clockwise);
}

ISR(PCINT2_vect) {
  if (digitalRead(rotary_encoder_sw) == LOW) 
  {
    buttonPressed = true;
    Serial.print("\nbutton pressed!\n");
   }
}
  

void setup()
{
  // put your setup code here, to run once:
 lcd.init();
 lcd.backlight();
 //lcd.createChar(0, rightArrow); 
 //lcd.createChar(0, upDownArrow);
 //screen1();
 //lcd.setCursor(0, 0);
 //lcd.write(byte(1)); // 0,2,4,6
 //lcd.write(byte(5));
 pinMode(rotary_encoder_clk, INPUT);
 pinMode(rotary_encoder_dt, INPUT);
 pinMode(rotary_encoder_sw, INPUT);
 PCICR |= 0b00000100;
 PCMSK2 |= 0b00010000;   // turn o PCINT20(D4)
 Serial.begin(9600);
 attachInterrupt(digitalPinToInterrupt(rotary_encoder_dt), isr0, RISING);
 screen0();
 delay(2500);
 screen1();
 lcd.setCursor(1,0);
 lcd.write(byte(62));
}

void loop() 
{
  if (turnDetected)
  {
    delay(200);
    switch (screen)
    {
      case 1:
      {
        if (clockwise)
        {
          lcd.setCursor(1,0);
          lcd.write(byte(32));
          lcd.setCursor(2,0);
          lcd.write(byte(62));
          powerMode = false;
        }
        else
        {
          lcd.setCursor(2,0);
          lcd.write(byte(32));
          lcd.setCursor(1,0);
          lcd.write(byte(62));
          powerMode = true;
        }
        break;
       case 2:
       {
          lcd.setCursor(cursorPosition, 0);
          lcd.write(byte(32));
          
          if (clockwise)
          { 
              cursorPosition = (cursorPosition + 1) % 3;
          }
          else
          {
              cursorPosition = (cursorPosition - 1) % 3;
              cursorPosition = cursorPosition < 0 ? 3 + cursorPosition : cursorPosition;
          }
          lcd.setCursor(cursorPosition, 0);
          lcd.write(byte(62));
       }
       break;
      }
    }
    turnDetected = false;
  }

  if (buttonPressed)
  {
    delay(200);
    switch (screen)
    {
      case 1:
      {
        if (powerMode)
        {
          screen3();
          screen = 3;
        }
        else
        {
          screen2();
          screen = 2;
        }
      }
      break;
      case 2:{}
      case 3:{}
      case 4:{}
      case 5:{}
      case 6:{}
      case 7:{}
    }
  }
}

void screen0() 
{
  lcd.clear();
  lcd.setCursor(0, 0); // Start of 1st row
  lcd.print("MONASH NOVA");
  lcd.setCursor(1, 0); // Start of 2nd row
  lcd.print("ROVER");
//  lcd.setCursor(24, 0);
//  lcd.setCursor(2, 10);
  lcd.setCursor(2, 0); // Start of 3rd row
  lcd.print("ELECTRONIC LOAD");
}

void screen1()
{
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Current Mode");
  lcd.setCursor(2, 1);
  lcd.print("Power Mode");
}

void screen2()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Current:");
  lcd.print(current);
  lcd.setCursor(0, 7);
  lcd.print("A");
  lcd.setCursor(1, 1);
  lcd.print("Next");
  lcd.setCursor(2, 1);
  lcd.print("Back");
  lcd.setCursor(0,0);
  lcd.write(byte(62));
}

void screen3()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Power:");
  lcd.setCursor(1, 7);
  lcd.print(power);
  lcd.print("W");
  lcd.setCursor(2, 1);
  lcd.print("Next");
  lcd.setCursor(1, 2);
  lcd.print("Back");
}

void screen4() 
{
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Current:");
  lcd.print(current);
  lcd.print("A");
}

void screen5() 
{
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Power:");
  lcd.print(power);
  lcd.print("W");
}

void screen6() 
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Data logging?");
  lcd.setCursor(1, 1);
  lcd.print("Yes");
  lcd.setCursor(1, 2);
  lcd.print("No");
  lcd.setCursor(1, 3);
  lcd.print("Back");
}

void screen7() 
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Low Voltage");
  lcd.setCursor(1, 1);
  lcd.print("Cut-off:");
  lcd.print(lowVolt);
  lcd.print("V");
  lcd.setCursor(1, 2);
  lcd.print("Start");
  lcd.setCursor(1, 3);
  lcd.print("Back");
}

void screen8() 
{
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Low Voltage");
  lcd.setCursor(1, 2);
  lcd.print("Cut-off:");
  lcd.print(lowVolt);
  lcd.print("V");
}

void screen9()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set:");
  lcd.print(current);
  lcd.print("A");
  lcd.setCursor(0, 1);
  lcd.print("Cur:");
  lcd.print(curCurrent);
  lcd.print("A");
  lcd.setCursor(0, 2);
  lcd.print("Temp:");
  lcd.print(hotTemp); 
  lcd.print("C");
  lcd.setCursor(0, 3);
  lcd.print("Time:");
  lcd.print(elapsedTime);
  lcd.print("s");
  lcd.setCursor(12, 3);
  lcd.write((uint8_t)0);
  lcd.print("END");
}

void screen10()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set:");
  lcd.print(power);
  lcd.print("W");
  lcd.setCursor(0, 1);
  lcd.print("Cur:");
  lcd.print(curPower);
  lcd.print("W");
  lcd.setCursor(0, 2);
  lcd.print("Temp:");
  lcd.print(hotTemp); 
  lcd.print("C");
  lcd.setCursor(0, 3);
  lcd.print("Time:");
  lcd.print(elapsedTime);
  lcd.print("s");
  lcd.setCursor(12, 3);
  lcd.write((uint8_t)0);
  lcd.print("END");
}

void screen11()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("NO/INVALID");
  lcd.setCursor(1, 1);
  lcd.print("SD CARD");
  lcd.setCursor(1, 2);
  lcd.write((uint8_t)0);
  lcd.print("Back");
}

void screen12()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("OVERTEMPERATURE");
  lcd.setCursor(5, 2);
  lcd.print("ERROR");
}

void screen13()
{
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("LOW VOLTAGE");
  lcd.setCursor(0, 2);
  lcd.print("CUT-OFF TERMINATION");
}

void screen14()
{
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("ANALYSIS");
  lcd.setCursor(2, 1);
  lcd.print("COMPLETE");
  lcd.setCursor(0,0);
  lcd.print("REMOVE SD CARD");
}
