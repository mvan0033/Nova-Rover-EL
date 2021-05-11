/* Program to implement the logic and user interface for an Ardunio Nano on the Monash Nova Rover Electronic Load.

Written by Matt van Wijk
Date: 11/05/2021
*/

// Initially set power limit to 100W
// Header Files
#include "LiquidCrystal_I2C.h"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include <SPI.h>

// Global variables
// Defining Pins
#define rotary_encoder_clk 2
#define rotary_encoder_dt 3
#define rotary_encoder_sw 4

volatile bool clockwise;
volatile bool turnDetected = false;
volatile bool buttonPressed = false;

// Variables for displaying power value
int powerDigit1 = 1; // Tens digit 
int powerDigit2 = 0; // Units digit
// int powerDigit3 = 0; // Tenths digit
// int powerDigit4 = 0; // Hundredth digit
// int powerDecimalPlace = 0;
int power;

// Variables for displaying current values
int currentDigit1 = 1; // Tens digit
int currentDigit2 = 0; // Units digit
// int currentDigit3 = 0; // Tenths digit
// int currentDigit4 = 0; // Hundredth digit
// int currentDecimalPlace = 0;
int current;

// Variables for displaying low voltage values
int lowVoltageDigit1 = 0; // Tens digit
int lowVoltageDigit2 = 0; // Units digit
int lowVoltageDigit3 = 0; // Tenths digit
int lowVoltageDecimalPlace = 0;
float lowVoltage;

// Misc Variables
float curCurrent = 0;
float curPower = 0;
float hotTemp = 0;
int elapsedTime = 0;
int lcdAddress = 0x27;
int screen = 1;
int cursorPosition = 0;
int selectedDigit = 0;
bool dataLogging = false; 
bool powerMode;
// bool nextDigit = false;

// Creating LCD object
LiquidCrystal_I2C lcd(lcdAddress, 128, 64);

// Start up screen
void screen0() 
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MONASH NOVA");
  lcd.setCursor(1, 0); 
  lcd.print("ROVER");
  lcd.setCursor(2, 0);
  lcd.print("ELECTRONIC LOAD");
}

// Current/power mode selection
void screen1()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Current Mode");
  lcd.setCursor(1, 1);
  lcd.print("Power Mode");
  lcd.setCursor(0, 0);
  lcd.write(byte(62)); // 62 is arrow character, 32 is empty character
  cursorPosition = 0;
}

// Setting current value
void screen2()
{
  // Calculate current value
  current = currentDigit1*pow(10, 1) + currentDigit2;
  
  // Set up screen
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Curr:");
  lcd.print(current);
  lcd.print("A");
  lcd.setCursor(1, 1);
  lcd.print("Next");
  lcd.setCursor(2, 1);
  lcd.print("Back");
  lcd.setCursor(0, 0);
  lcd.write(byte(62)); // Print an arrow character
  cursorPosition = 0;
}

// Setting power value
void screen3()
{
  // Calculate power value
  power = powerDigit1*pow(10, 1) + powerDigit2;
  
  // Set up screen
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Power:");
  lcd.print(power);
  lcd.print("W");
  lcd.setCursor(1, 1);
  lcd.print("Next");
  lcd.setCursor(2, 1);
  lcd.print("Back");
  lcd.setCursor(0, 0);
  lcd.write(byte(62)); // Print an arrow character
  cursorPosition = 0;
}

// Adjusting current
void screen4() 
{
  // Calculate current value
  current = currentDigit1*pow(10, 1) + currentDigit2;

  // Set up screen
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Curr:");

  // Padding if necessary
  if (current < 10)
  {
    lcd.print("0");
    lcd.print(current);
  }
  else
  {
    lcd.print(current);
  }

  lcd.print("A");
}

// Adjusting power
void screen5() 
{
  // Calculate power value
  power = powerDigit1*pow(10, 1) + powerDigit2;

  // Set up screen
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Power:");

  // Padding if necessary
  if (power < 10)
  {
    lcd.print("0");
    lcd.print(power);
  }
  else
  {
    lcd.print(power);
  }

  lcd.print("W");
}

// Prompt user for data logging
void screen6() 
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Data logging?");
  lcd.setCursor(1, 1);
  lcd.print("Yes");
  lcd.setCursor(2, 1);
  lcd.print("No");
  lcd.setCursor(3, 1);
  lcd.print("Back");
  lcd.setCursor(1, 0);
  lcd.write(byte(62)); // Print an arrow character
  cursorPosition = 0;
}

// Prompt user for low-voltage cut-off
void screen7() 
{
  // Calculate low voltage value
  lowVoltage = lowVoltageDigit1*pow(10, 1) + lowVoltageDigit2 + lowVoltageDigit3*pow(10, -1);

  // Set up screen
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Low Voltage");
  lcd.setCursor(1, 1);
  lcd.print("Cut-off:");
  lcd.print(lowVoltage, 1);
  lcd.print("V");
  lcd.setCursor(2, 1);
  lcd.print("Start");
  lcd.setCursor(3, 1);
  lcd.print("Back");
  lcd.setCursor(0, 0);
  lcd.write(byte(62)); // Print an arrow character
  cursorPosition = 0;
}

// Adjusting low-voltage cut-off
void screen8() 
{
  // Calculate low voltage value
  lowVoltage = lowVoltageDigit1*pow(10, 1) + lowVoltageDigit2 + lowVoltageDigit3*pow(10, -1);

  // Set up screen
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Low Voltage");
  lcd.setCursor(1, 1);
  lcd.print("Cut-off:");
  lcd.setCursor(2, 1);

  // Padding the number
  if (lowVoltage < 10)
  {
      lcd.print("0");
      lcd.print(lowVoltage, 1);
  }
  else
  {
      lcd.print(lowVoltage, 1);
  }

  lcd.print("V");
}

// Display system monitor for current
void screen9()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set:");
  lcd.print(current);
  lcd.print("A");
  lcd.setCursor(1, 0);
  lcd.print("Cur:");
  lcd.print(curCurrent);
  lcd.print("A");
  lcd.setCursor(2, 0);
  lcd.print("Temp:");
  lcd.print(hotTemp); 
  lcd.print("C");
  lcd.setCursor(3, 0);
  lcd.print("Time:");
  lcd.print(elapsedTime);
  lcd.print("s");
  lcd.setCursor(3, 6);
  lcd.write(byte(62));
  lcd.print("END");
}

// Display system monitor for power
void screen10()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set:");
  lcd.print(power);
  lcd.print("W");
  lcd.setCursor(1, 0);
  lcd.print("Cur:");
  lcd.print(curPower);
  lcd.print("W");
  lcd.setCursor(2, 0);
  lcd.print("Temp:");
  lcd.print(hotTemp); 
  lcd.print("C");
  lcd.setCursor(3, 0);
  lcd.print("Time:");
  lcd.print(elapsedTime);
  lcd.print("s");
  lcd.setCursor(3, 6);
  lcd.write(byte(62));
  lcd.print("END");
}

// No/invaild SD card
void screen11()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("NO/INVALID");
  lcd.setCursor(1, 1);
  lcd.print("SD CARD");
  lcd.setCursor(1, 2);
  lcd.write(byte(62));
  lcd.print("Back");
}

// Overtemperature error
void screen12()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("OVERTEMPERATURE");
  lcd.setCursor(5, 2);
  lcd.print("ERROR");
}

// Low-voltage error
void screen13()
{
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("LOW VOLTAGE");
  lcd.setCursor(2, 0);
  lcd.print("CUT-OFF TERMINATION");
}

// Analysis complete
void screen14()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ANALYSIS");
  lcd.setCursor(1, 0);
  lcd.print("COMPLETE");
  lcd.setCursor(2, 0);
  lcd.print("REMOVE SD CARD");
}

// Early termination
void screen15()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ANALYSIS");
  lcd.setCursor(1, 0);
  lcd.print("TERMINATED");
}

// Function to display set the current/power and display on the LCD
void setPowerCurrent(int powerCurrent)
{
  
}

/* Function for moving the cursor position 
 
  :pre: 0 <= numOptions <= 4
  :pre: 0 <= rowOffset <= 4
  :param numOptions: is the number of options to be displayed for the current screen
  :param rowOffset: is the row number for the first option
  :param row1Overflow: Boolean variable for a screen with its first option spilling over two rows
  :post: the cursor is moved to its next position and printed to the LCD screen
*/
void optionSelection(int numOptions, int rowOffset, bool row1Overflow)
{
  if (row1Overflow && cursorPosition == 0)
  {
    lcd.setCursor(0, 0);
    lcd.write(byte(32)); // Empty character
  }
  else
  {
    lcd.setCursor(cursorPosition + rowOffset, 0);
    lcd.write(byte(32)); // Empty character
  }

  
  if (clockwise)
  { 
      cursorPosition = (cursorPosition + 1) % numOptions;
  }
  else
  {
      cursorPosition = (cursorPosition + numOptions - 1) % numOptions;
  }

  if (row1Overflow && cursorPosition == 0)
  {
    lcd.setCursor(0, 0);
    lcd.write(byte(62)); // Arrow character
  }
  else
  {
    lcd.setCursor(cursorPosition + rowOffset, 0);
    lcd.write(byte(62)); // Arrow character
  }
}

/* Function to flash a digit that indicates the currently selected digit

  :pre: 0 <= row <= 4
  :pre: 0 <= col <= 8
  :param digit: the digit to be flashed
  :param row: the row of the digit on the LCD screen
  :param col: the column of the digit on the LCD screen
  :param string: a string that is printed to the LCD screen either before the digit is flashed or after
  :param before: Boolean variable that specifes whether string is printed before or after (true = before, false = after)
  :post: the function will be stuck in a loop, flashing digit until either a turn is detected or a button pressed
*/
void flashDigit(int digit, int row, int col, const char *string, bool before)
{
  bool digitFlashIteration = true;

  // Looping until a turn is detected or a button pressed
  while (!turnDetected && !buttonPressed)
  { 
    // Printing the digit to LCD for this iteration
    if (digitFlashIteration)
    {
      lcd.setCursor(row, col);

      if (before)
      {
        lcd.print(string);
        lcd.print(digit);
      }
      else
      {
        lcd.print(digit);
        lcd.print(string);
      }

      digitFlashIteration = false;
    }

    // Printing an empty space to LCD for this iteration
    else
    {
      lcd.setCursor(row, col);

      if (before)
      {
        lcd.print(string);
        lcd.print(" "); // Print an empty space 
      }
      else
      {
        lcd.print(" "); // Print an empty space
        lcd.print(string);
      }

      digitFlashIteration = true;
    }

    delay(250);
  }

  // Printing the digit to LCD
  lcd.setCursor(row, col);
  if (before)
  {
    lcd.print(string);
    lcd.print(digit);
  }
  else
  {
    lcd.print(digit);
    lcd.print(string);
  }
}

/* Function to increment or decrement a specific digit when setting power/current/low-voltage values

  :pre: digitToChange is a pointer to an integer digit 
  :param digitToChange: a pointer to the currently selected digit that is about to be incremented/decremented
  :post: digitToChange is modified such that it is either incremented or decremented (depending on turn direction)
*/
void changeDigit(int *digitToChange)
{
    if (clockwise) // If turn is clockwise
    {
      *digitToChange = (*digitToChange + 1) % 10; // Increment
    }
    else
    {
      *digitToChange = (*digitToChange + 9) % 10; // Decrement
    } 
}

// ISR for when a turn is detected from the rotary encoder
void isr0()
{
  turnDetected = true;

  // Check direction
  clockwise = digitalRead(rotary_encoder_clk) != digitalRead(rotary_encoder_dt);
}

// ISR for when the button on the rotary encoder is pressed
ISR(PCINT2_vect)
{
  if (digitalRead(rotary_encoder_sw) == LOW) 
  {
    buttonPressed = true;
  }
}
  
void setup()
{
 // Initalising LCD
 lcd.init();
 lcd.backlight();

 // Initalising pins
 pinMode(rotary_encoder_clk, INPUT_PULLUP);
 pinMode(rotary_encoder_dt, INPUT_PULLUP);
 pinMode(rotary_encoder_sw, INPUT_PULLUP);

 // Setting up ISR for rotary encoder button
 PCICR |= 0b00000100;
 PCMSK2 |= 0b00010000;   // turn on PCINT20(D4)
 
 // Setting up serial monitor
 Serial.begin(9600);

 // Setting up ISR for turning event 
 attachInterrupt(digitalPinToInterrupt(rotary_encoder_dt), isr0, RISING);

 // Beginning the LCD start sequence
 screen0();
 delay(2500);
 screen1();
 lcd.setCursor(0, 0);
 lcd.write(byte(62)); // 62 is arrow character, 32 is empty character
}

void loop() 
{
  delay(60);
  if (turnDetected)
  {
    delay(50);
    switch (screen)
    {
     case 1: // Current/Power selection screen
     {
        // Change selected option
        optionSelection(2, 0, false);
        break;
      }
     
     case 2: // Current mode menu screen
     {
        // Change selected option
        optionSelection(3, 0, false);
        break;
     }
     
     case 3: // Power mode menu screen
     {
        // Change selected option
        optionSelection(3, 0, false);
        break;
     }

     case 4: // Adjusting current screen
     { 
        switch (selectedDigit)
        {
          case 1:
          {
            changeDigit(&currentDigit1);
            screen4();
            break;
          }
          case 2:
          {
            changeDigit(&currentDigit2);
            screen4();
            break;
          }
          default: 
          {
            Serial.print("1. This is an error\n");
          }
        }
        break;   
     }
     
     case 5: // Adjusting power screen
     {
       // Change digit
       switch (selectedDigit)
        {
          case 1:
          {
            changeDigit(&powerDigit1);
            screen5();
            break;
          }
          case 2:
          {
            changeDigit(&powerDigit2);
            screen5();
            break;
          }
          default:
          {
            Serial.print("2. This is an error\n");
          }
        }
        break;   
     }
     
     case 6: // Data logging? screen
     {
       // Change selected option
       optionSelection(3, 1, false);
       break;
     }

     case 7: // Low voltage cut-off screen
     {
        // Change selected option
        optionSelection(3, 1, true); 
        break; 
     }

     case 8: // Low voltage cut-off adjustment screen 
     {
        // Change digit
        switch (selectedDigit)
        {
          case 1:
          {
            changeDigit(&lowVoltageDigit1);
            screen8();
            break;
          }
          case 2:
          {
            changeDigit(&lowVoltageDigit2);
            screen8();
            break;
          }
          case 3:
          {
            changeDigit(&lowVoltageDigit3);
            screen8();
            break;
          }
          default:
          {
            Serial.print("3. This is an error\n");
          }
        }
        break;
     }

     case 9: // Current monitor screen
     {
       // Do nothing
       break;
     }

     case 10: // Power monitor screen
     {
       // Do nothing
       break;
     }

     case 11: // No/invaid SD card screen
     {
       // Do nothing
       break;
     }

     case 12: // Overtemperature screen
     {
       // Do nothing
       break;
     }

     case 13: // Low-voltage cut-off termination screen
     {
       // Do nothing
       break;
     }
     
     case 14: // Analysis complete screen
     {
       // Do nothing
       break;
     }

     case 15: // Analysis terminated screen
     {
       // Do nothing
       break;
     }
    }

    // Reset turnDetected
    turnDetected = false;
  }
  
  if (buttonPressed)
  {
    // Reset buttonPressed
    buttonPressed = false;
    delay(200);

    switch (screen)
    {
      case 1: // Current/Power selection screen
      {
        // If power mode is selected
        if (cursorPosition == 1) 
        {
          screen3();
          screen = 3;
          powerMode = true;
        }
        // If current mode is selected
        else 
        {
          screen2();
          screen = 2;
          powerMode = false;
        }
        break;
      }
      
      case 2: // Current mode screen
      {
        switch (cursorPosition)
        {
          case 0: // Set current
          {
            // Display current changing screen
            screen4();
            screen = 4; 
            break;
          }
          case 1: // Next
          {
            screen6();
            screen = 6;
            break;
          }
          case 2: // Back 
          {
            screen1();
            screen = 1;
            break;
          }
          break;
        }
        break;
      }
      
      case 3: // Power mode screen
      {
        switch (cursorPosition)
        {
          case 0: // Set power
          {
            screen5();
            screen = 5; 
            break;
          }
          case 1: // Next
          {
            screen6();
            screen = 6;
            break;
          }
          case 2: // Back 
          {
            screen1();
            screen = 1;
            break;
          }
        }
        break;
      }

      case 4: // Change current screen
      {
        // Flash digit & move along
        selectedDigit++;

        switch (selectedDigit)
        {
          case 1: // Tens digit
          {
            flashDigit(currentDigit1, 1, 3, ":", true);
            break;
          }
          case 2: // Units digit
          {
            flashDigit(currentDigit2, 1, 4, "A", false);
            break;
          }
          default: // Continue to next screen and reset selectedDigit
          {
            screen2();
            screen = 2;
            selectedDigit = 0;
          }
        }
        break;
      }

      case 5: // Change power screen
      {
        // Flash digit & move along
        selectedDigit++;

        switch (selectedDigit)
        {
          case 1: // Tens digit
          {
            char buffer[2];
            itoa(powerDigit2, buffer, 10);
            flashDigit(powerDigit1, 1, 4, buffer, false);
            break;
          }
          case 2: // Units digit
          {
            char buffer[2];
            itoa(powerDigit1, buffer, 10);
            flashDigit(powerDigit2, 1, 4, buffer, true);
            break;
          }
          // case 3: // Tenths digit
          // {
          //   flashDigit(currentDigit3, 1, 5, "", true);
          //   break;
          // }
          // case 4: // Hundredth digit
          // {
          //   char buffer[2];
          //   itoa(currentDigit3, buffer, 10);
          //   flashDigit(currentDigit4, 1, 6, "W", false);
          //   break;
          // }
          default: // Continue to next screen and reset selectedDigit
          {
            screen3();
            screen = 3;
            selectedDigit = 0;
          }
        }
        break;
      }

      case 6: // Data logging? screen
      {
        switch (cursorPosition)
        {
          case 0: // Yes
          {
            screen7();
            screen = 7;
            dataLogging = true;
            break;
          }
          case 1: // No
          {
            screen7();
            screen = 7;
            break;
          }
          case 2: // Back
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
            break;
          }
        }
        break;
      }

      case 7: // Low voltage cut-off screen
      {
        switch (cursorPosition)
        {
          case 0: // Set Low voltage cut-off
          {
            screen8();
            screen = 8;
            break;
          }
          
          case 1: // Start
          {
            if (powerMode)
            {
              screen10();
              screen = 10;
            }
            else
            {
              screen9();
              screen = 9;
            }
            break;
           }

          case 2: // Back
          {
            screen6();
            screen = 6;
            break;
          }
        }
        break;
      }
    
      case 8: // Low voltage cut-off adjustment screen 
      {
        // Flash digit & move along
        selectedDigit++;

        switch (selectedDigit)
        {
          case 1: // Tens digit
          {
            char buffer[2];
            itoa(lowVoltageDigit2, buffer, 10);
            flashDigit(lowVoltageDigit1, 2, 1, buffer, false);
            break;
          }
          case 2: // Units digit
          {
            char buffer[2];
            itoa(lowVoltageDigit1, buffer, 10);
            flashDigit(lowVoltageDigit2, 2, 1, buffer, true);
            break;
          }
          case 3: // Tenths digit
          {
            flashDigit(lowVoltageDigit3, 2, 2, ".", true);
            break;
          }
          default: // Continue to next screen and reset selectedDigit
          {
            screen7();
            screen = 7;
            selectedDigit = 0;
          }
        }
        break;
      }
    
      case 9: // Current monitor screen
      {
        screen15();
        screen = 15;

        // TODO: End termination
        break;
      }

      case 10: // Power monitor screen
      {
        screen15();
        screen = 15;
        
        // TODO: End termination
        break;
      }

      case 11: // No/invaid SD card screen
      {
        screen6();
        screen = 6;
        break;
      }

      case 12: // Overtemperature screen
      {
        screen1();
        screen = 1;
        break;
      }

      case 13: // Low-voltage cut-off termination screen
      {
        screen1();
        screen = 1;
        break;
      }

      case 14: // Analysis complete screen
      {
        screen1();
        screen = 1;
        break;
      }

      case 15: // Analysis terminated screen
      {
        screen1();
        screen = 1;
        break;
      }
    }
  }
}


