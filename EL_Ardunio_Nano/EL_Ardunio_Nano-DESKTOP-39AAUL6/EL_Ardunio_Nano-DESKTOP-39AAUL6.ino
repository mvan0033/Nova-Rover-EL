// Importing
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Global variables
float power = 0;
float current = 0;
float curCurrent = 0;
float curPower = 0;
float lowVolt = 0;
float hotTemp = 0;
int elaspsedTime = 0;
int lcdAddress = 0x27;

// Custom LCD Characters
byte rightArrow[8] = 
{
  0b10000,
  0b11000,
  0b11100,
  0b11110,
  0b11110,
  0b11100,
  0b11000,
  0b10000
};

byte upDownArrow[8] = 
{
  0b00100,
  0b01110,
  0b11111,
  0b00000,
  0b00000,
  0b11111,
  0b01110,
  0b00100,
};

void setup()
{
  // put your setup code here, to run once:
 LiquidCrystal_I2C lcd(lcdAdd, 128, 64);
 lcd.begin();
 lcd.createChar(0, rightArrow); 
 lcd.createChar(1, upDownArrow);
}

void loop() 
{
  // put your main code here, to run repeatedly:

}

void screen0() 
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("MONASH NOVA");
  lcd.setCursor(3, 1);
  lcd.print("ROVER");
  lcd.setCursor(0,3);
  lcd.print("ELECTRONIC LOAD");
}

void screen1()
{
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Current Mode");
  lcd.setCursor(1, 2);
  lcd.print("Power Mode");
}

void screen2()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Current:");
  lcd.print(current);
  lcd.print("A");
  lcd.print(1, 1);
  lcd.print("Next");
  lcd.setCursor(1, 2);
  lcd.print("Back");
}

void screen3()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Power:");
  lcd.print(power);
  lcd.print("W");
  lcd.print(1, 1);
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
  lcd.setCursor(0, 2);
  lcd.print("REMOVE SD CARD");
}
