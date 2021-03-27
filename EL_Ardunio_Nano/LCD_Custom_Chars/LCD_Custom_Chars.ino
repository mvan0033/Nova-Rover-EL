// Importing
#include "LiquidCrystal_I2C.h"
#include <Wire.h>

int lcdAddress = 0x27;
LiquidCrystal_I2C lcd(lcdAddress, 128, 64);

void setup() {
  // put your setup code here, to run once:
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("A");
  
}
// 16 --> right arrow
// 31 --> down arrow
// 32 --> up arrow

void loop() {
  // put your main code here, to run repeatedly:
  
  
  
    for (int j = 2; j < 128; j++)
    {
      char buffer[16];
      lcd.clear();
      lcd.setCursor(0, 0);
      // lcd.write(byte(i));
      lcd.write(byte(j));
      itoa(j, buffer, 10);
      lcd.setCursor(3, 0);
      lcd.print("Number: ");
      lcd.setCursor(3, 1);
      lcd.print(buffer);
      delay(750);
    
  }
  
}
