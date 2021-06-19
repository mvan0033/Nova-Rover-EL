/* Program to implement the logic and user interface for an Ardunio Nano on the Monash Nova Rover Electronic Load.

Written by Matt van Wijk
Last modified: 19/06/2021
*/

// Initially set power limit to 100W
// Header Files
#include "LiquidCrystal_I2C.h"
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include "../../EL_ADC_PWM_Module/headers/control_loop.h"

// Control loop object which handles our PWM/ADC hardware.
ControlLoop controller;

// RTC object which handles RTC hardware
RTC_PCF8523 rtc;
DateTime now;

// SD card object which handles SD Card hardware
File dataFile;

// Global variables
// Defining Pins
#define rotary_encoder_clk 2
#define rotary_encoder_dt 3
#define rotary_encoder_sw 4
#define buzzer_pin 7

volatile bool clockwise;
volatile bool turnDetected;
volatile bool buttonPressed;

// Variables for displaying power value
int powerDigit1 = 1; // Tens digit
int powerDigit2;     // Units digit
// int powerDigit3 = 0; // Tenths digit
// int powerDigit4 = 0; // Hundredth digit
// int powerDecimalPlace = 0;
int power;

// Variables for displaying current values
int currentDigit1 = 1; // Tens digit
int currentDigit2;     // Units digit
// int currentDigit3 = 0; // Tenths digit
// int currentDigit4 = 0; // Hundredth digit
// int currentDecimalPlace = 0;
int current;

// Variables for displaying low voltage values
int timeLimitDigit1; // Hundreds digit
int timeLimitDigit2; // Tens digit
int timeLimitDigit3 = 1; // Units digit
float timeLimit = 1;

// Variables for displaying low voltage values
int lowVoltageDigit1; // Tens digit
int lowVoltageDigit2; // Units digit
int lowVoltageDigit3; // Tenths digit
double lowVoltage;

// Misc Variables
double curCurrent;
double curPower;
double hotTemp;
const double MAX_TEMPERATURE = 50;
int elapsedTime;
int lcdAddress = 0x27;
int screen = 1;
int cursorPosition;
int selectedDigit;
bool dataLogging;
bool powerMode;
unsigned long referenceTime = millis();
unsigned long timeOffset;

// Creating LCD object
LiquidCrystal_I2C lcd(lcdAddress, 128, 64);

// Buzzer function
void buzzer()
{
  pinMode(buzzer_pin, OUTPUT);

  while (!buttonPressed)
  {
    digitalWrite(buzzer_pin, HIGH);
    delay(2);
    digitalWrite(buzzer_pin, LOW);
    delay(2);
  }
}

// Start up screen
void screen0(uint8_t minute, uint8_t hour, uint8_t day, uint8_t month, int16_t year)
{
  screen = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MONASH NOVA");
  lcd.setCursor(1, 0);
  lcd.print("ROVER");
  lcd.setCursor(2, 0);
  lcd.print("ELECTRONIC LOAD");
  lcd.setCursor(3, 0);
  lcd.print(hour);
  lcd.print(":");

  // Padding the minutes if necessary
  if (minute < 10)
  {
    lcd.print("0");
  }
  lcd.print(minute);
  
  lcd.print(" ");
  lcd.print(day);
  lcd.print("/");
  lcd.print(month);
  lcd.print("/");
  lcd.print(year);
}

// Current/power mode selection
void screen1()
{
  screen = 1;
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
  screen = 2;

  // Calculate current value
  current = currentDigit1 * pow(10, 1) + currentDigit2;

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
  screen = 3;

  // Calculate power value
  power = powerDigit1 * pow(10, 1) + powerDigit2;

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
  screen = 4;

  // Calculate current value
  current = currentDigit1 * pow(10, 1) + currentDigit2;

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
  screen = 5;

  // Calculate power value
  power = powerDigit1 * pow(10, 1) + powerDigit2;

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
  screen = 6;
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

// Prompt user for time limit
void screen7()
{
  screen = 7;

  // Calculate time limit value
  timeLimit = timeLimitDigit1 * pow(10, 2) + timeLimitDigit2 * pow(10, 1) + timeLimitDigit3;

  // Set up screen
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(timeLimit, 0);
  lcd.print("mins");
  lcd.setCursor(1, 1);
  lcd.print("Next");
  lcd.setCursor(2, 1);
  lcd.print("Back");
  lcd.setCursor(0, 0);
  lcd.write(byte(62)); // Print an arrow character
  cursorPosition = 0;
}

// Adjusting time limit
void screen8()
{
  screen = 8;

  // Calculate time limit value
  timeLimit = timeLimitDigit1 * pow(10, 2) + timeLimitDigit2 * pow(10, 1) + timeLimitDigit3;

  // Set up screen
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Time Limit: ");
  lcd.setCursor(1, 1);

  // Padding the number
  if (timeLimit < 10)
  {
    lcd.print("00");
    lcd.print(timeLimit, 0);
  }
  else if (timeLimit < 100)
  {
    lcd.print("0");
    lcd.print(timeLimit, 0);
  }
  else
  {
    lcd.print(timeLimit, 0);
  }

  lcd.print("mins");
}

// Prompt user for low-voltage cut-off
void screen9()
{
  screen = 9;

  // Calculate low voltage value
  lowVoltage = lowVoltageDigit1 * pow(10, 1) + lowVoltageDigit2 + lowVoltageDigit3 * pow(10, -1);

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
void screen10()
{
  screen = 10;

  // Calculate low voltage value
  lowVoltage = lowVoltageDigit1 * pow(10, 1) + lowVoltageDigit2 + lowVoltageDigit3 * pow(10, -1);

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
void screen11()
{
  screen = 11;

  // Set the current mode and output
  controller.set_target_mode(0);
  controller.set_target_value(current);

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

  controller.set_enable(true);
}

// Display system monitor for power
void screen12()
{
  screen = 12;

  // Set to power mode and output
  controller.set_target_mode(1);
  controller.set_target_value(power);

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

  controller.set_enable(true);
}

// No/invaild SD card
void screen13()
{
  screen = 13;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("NO/INVALID");
  lcd.setCursor(1, 0);
  lcd.print("SD CARD");
  lcd.setCursor(2, 0);
  lcd.write(byte(62));
  lcd.print("Back");
}

// Overtemperature error
void screen14()
{
  screen = 14;
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("OVERTEMPERATURE");
  lcd.setCursor(2, 2);
  lcd.print("ERROR");

  // Play sound to indicate overtemperature error
  buzzer();
}

// Low-voltage error
void screen15()
{
  screen = 15;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LOW VOLTAGE");
  lcd.setCursor(1, 0);
  lcd.print("CUT-OFF");
  lcd.setCursor(2, 0);
  lcd.print("TERMINATION");

  // Play sound to indicate low-voltage error
  buzzer();
}

// Analysis complete
void screen16()
{
  screen = 16;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ANALYSIS");
  lcd.setCursor(1, 0);
  lcd.print("COMPLETE");
  lcd.setCursor(2, 0);
  lcd.print("REMOVE SD CARD");
  elapsedTime = 0;

  // Close dataFile
  if (dataLogging)
  {
    dataFile.close();
  }

  // Play sound to indicate analysis complete
  buzzer();
}

// Early termination
void screen17()
{
  screen = 17;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ANALYSIS");
  lcd.setCursor(1, 0);
  lcd.print("TERMINATED");

  // Close dataFile
  if (dataLogging)
  {
    dataFile.close();
  }

  // Play sound to indicate early termination
  buzzer();
}

// Confirm early termination
void screen18()
{
  screen = 18;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Terminate");
  lcd.setCursor(1, 0);
  lcd.print("Analysis?");
  lcd.setCursor(2, 1);
  lcd.print("Yes");
  lcd.setCursor(3, 1);
  lcd.print("No");
  lcd.setCursor(2, 0);
  lcd.write(byte(62)); // Print an arrow character
  cursorPosition = 0;
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

// // Function to log system data to external SD card
void writeDataToSD()
{
  // TODO: Print data to SD card
  dataFile.print(elapsedTime);
  dataFile.print(", ");
  dataFile.print(controller.get_total_voltage());
  dataFile.print(", ");
}

// Function to initialise SD Card 
void initialiseSD()
{  
  // See if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT_PIN)) 
  {
    // Print to console
    Serial.println("Card failed, or not present");

    // NO/INVALID SD CARD screen
    screen13();
  }
  else
  {
    // SD is present and working 
    Serial.println("card initialized.");

    // Create DateTime object for printing/displaying date & time
    now = rtc.now();

    // Format filename for SD card
    char filename[20];
    sprintf(filename, "Time_%d-%d__Date_%d-%d-%d", int(now.hour()), int(now.minute()), int(now.day()), int(now.month()), int(now.year()));
    Serial.println(filename);
  
    // Open file to write to
    dataFile = SD.open(filename, FILE_WRITE);

    // Check for errors opening file
    if (!dataFile) 
    {
      Serial.println("error opening datalog.txt");
      screen13();
    }
    else
    {
      dataLogging = true;

      // Print data headings
      dataFile.println("Time (s), System Voltage (V), Voltage1 (V), Voltage2 (V), Voltage3 (V), Mode, Mode Target, Mode Value, " 
      "Temperature1 (C), Temperature2 (C), Temperature3 (C), Temperature4 (C), Current1 (A), Current2 (A), Current3 (A), Current4 (A), PWM1, PWM2, "
      "PWM3, PWM4");

      // Move to next screen
      screen7();
    }
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
// PCINT2_vect
//#define PORTC_PORT_vect 0x30
// ISR(PORTC_PORT_vect)
// {
//   if (digitalRead(rotary_encoder_sw) == LOW)
//   {
//     buttonPressed = true;
//   }
// }
void isr1()
{
  if (digitalRead(rotary_encoder_sw) == LOW)
  {
    buttonPressed = true;
  }
}


void setup()
{
  Serial.begin(9600);

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (!rtc.initialized() || rtc.lostPower())
  {
    Serial.println("Set time for RTC");
    // Set RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // Start RTC
  rtc.start();

  // Create DateTime object for printing/displaying date & time
  now = rtc.now();

  // Setup control loop object (which initialises ADC/PWM hardware)
  controller.init();

  // Initialising LCD
  lcd.init();
  lcd.backlight();

  // Initialising pins for rotary encoder
  pinMode(rotary_encoder_clk, INPUT_PULLUP);
  pinMode(rotary_encoder_dt, INPUT_PULLUP);
  pinMode(rotary_encoder_sw, INPUT_PULLUP);

  // Initialising pins for SD card
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);

  // Setting up ISR for rotary encoder button
  // PCICR |= 0b00000100;
  // PCMSK2 |= 0b00010000; // turn on PCINT20(D4)
  // PORTC.PIN6CTRL |= 0b00000010;
  // #define WRITE_PIN6CTRL(val) ((*(volatile uint32_t *)0x16) = (val));
  // WRITE_PIN6CTRL(0b00000010); // set pin 6 as interrupt on rising edge

  // Setting up serial monitor
  Serial.begin(9600);

  // Setting up ISR for turning event
  attachInterrupt(digitalPinToInterrupt(rotary_encoder_dt), isr0, RISING);
  attachInterrupt(digitalPinToInterrupt(rotary_encoder_sw), isr1, CHANGE);

  // Beginning the LCD start sequence
  screen0(now.minute(), now.hour(), now.day(), now.month(), now.year());
  delay(4000);
  screen1();

  // Initialise EL as off
  controller.set_enable(false);
}

void loop()
{
  // Update control loop!
  controller.update();

  // Check error state (true means error)
  if (controller.get_error_state())
  {
    Serial.print("Error on MODULE #");
    Serial.println(controller.get_error_module());
    Serial.print("Error type was #");
    Serial.println(controller.get_error_type());

    // Serial.println("Resetting...");
    // delay(5000);
    // controller.reset_errors();
  }

  // Update global variables every 1 seconds
  if (millis() - referenceTime > 1000)
  {
    // Update variables for system monitor
    curCurrent = controller.get_total_current();
    curPower = controller.get_total_power();
    elapsedTime = int(millis() / 1000) - int(timeOffset / 1000);
    hotTemp = controller.get_highest_temperature();
    
    if (screen == 11)
    {
      screen11();
    }
    else if (screen == 12)
    {
      screen12();
    }

    // Check elapsed time against time limit
    if (elapsedTime / 60.0 >= timeLimit && controller.get_pwm_active_state())
    {
      // Terminate Analaysis
      controller.set_enable(false);

      // Analysis complete screen
      screen16();
    }

    // Check system voltage against low voltage limit
    else if (controller.get_total_voltage() < lowVoltage && controller.get_pwm_active_state())
    {
      // Terminate Analaysis
      controller.set_enable(false);
      
      // Low voltage cut-off screen
      screen15();
    }

    // Check temperature against max temperature limit
    else if (controller.get_highest_temperature() > MAX_TEMPERATURE && controller.get_pwm_active_state())
    {
      // Terminate Analaysis
      controller.set_enable(false);

      // Overtemperature error screen
      screen14();
    }

    // Update reference time
    referenceTime = millis();
  }

  if (turnDetected)
  {
    // delay(50);
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

    case 7: // Time limit screen
    {
      // Change selected option
      optionSelection(3, 0, false);
      break;
    }

    case 8: // Time limit adjustment screen
    {
      // Change digit
      switch (selectedDigit)
      {
      case 1:
      {
        changeDigit(&timeLimitDigit1);
        screen8();
        break;
      }
      case 2:
      {
        changeDigit(&timeLimitDigit2);
        screen8();
        break;
      }
      case 3:
      {
        changeDigit(&timeLimitDigit3);
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

    case 9: // Low voltage cut-off screen
    {
      // Change selected option
      optionSelection(3, 1, true);
      break;
    }

    case 10: // Low voltage cut-off adjustment screen
    {
      // Change digit
      switch (selectedDigit)
      {
      case 1:
      {
        changeDigit(&lowVoltageDigit1);
        screen10();
        break;
      }
      case 2:
      {
        changeDigit(&lowVoltageDigit2);
        screen10();
        break;
      }
      case 3:
      {
        changeDigit(&lowVoltageDigit3);
        screen10();
        break;
      }
      default:
      {
        Serial.print("3. This is an error\n");
      }
      }
      break;
    }

    case 11: // Current monitor screen
    {
      // Do nothing
      break;
    }

    case 12: // Power monitor screen
    {
      // Do nothing
      break;
    }

    case 13: // No/invaid SD card screen
    {
      // Do nothing
      break;
    }

    case 14: // Overtemperature screen
    {
      // Do nothing
      break;
    }

    case 15: // Low-voltage cut-off termination screen
    {
      // Do nothing
      break;
    }

    case 16: // Analysis complete screen
    {
      // Do nothing
      break;
    }

    case 17: // Analysis terminated screen
    {
      // Do nothing
      break;
    }
    
    case 18: // Confirm terminate analysis screen
    {
      // Change selected option
      optionSelection(2, 2, false);
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
    // delay(200);

    switch (screen)
    {
    case 1: // Current/Power selection screen
    {
      // If power mode is selected
      if (cursorPosition == 1)
      {
        screen3();
        powerMode = true;
      }
      // If current mode is selected
      else
      {
        screen2();
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
        break;
      }
      case 1: // Next
      {
        screen6();
        break;
      }
      case 2: // Back
      {
        screen1();
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
        break;
      }
      case 1: // Next
      {
        screen6();
        break;
      }
      case 2: // Back
      {
        screen1();
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
        initialiseSD();
        break;
      }
      case 1: // No
      {
        screen7();
        break;
      }
      case 2: // Back
      {
        if (powerMode)
        {
          screen3();
        }
        else
        {
          screen2();
        }
        break;
      }
      }
      break;
    }

    case 7: // Time limit screen
    {
      switch (cursorPosition)
      {
      case 0: // Set Low voltage cut-off
      {
        screen8();
        break;
      }

      case 1: // Next
      {
        screen9();
        break;
      }

      case 2: // Back
      {
        screen6();
        break;
      }
      }
      break;
    }

    case 8: // Time limit adjustment screen
    {
      // Flash digit & move along
      selectedDigit++;

      switch (selectedDigit)
      {
      case 1: // Hundreds digit
      {
        char buffer[2];
        itoa(timeLimitDigit2, buffer, 10);
        flashDigit(timeLimitDigit1, 1, 1, buffer, false);
        break;
      }
      case 2: // Tens digit
      {
        char buffer[2];
        itoa(timeLimitDigit1, buffer, 10);
        flashDigit(timeLimitDigit2, 1, 1, buffer, true);
        break;
      }
      case 3: // Unit digit
      {
        flashDigit(timeLimitDigit3, 1, 2, "", true);
        break;
      }
      default: // Continue to next screen and reset selectedDigit
      {
        screen7();
        selectedDigit = 0;
      }
      }
      break;
    }

    case 9: // Low voltage cut-off screen
    {
      switch (cursorPosition)
      {
      case 0: // Set Low voltage cut-off
      {
        screen10();
        break;
      }

      case 1: // Start
      {

        // Set the time offset for the elaspsed time
        timeOffset = millis();
        elapsedTime = int(millis() / 1000) - int(timeOffset / 1000);

        if (powerMode)
        {
          screen12();
        }
        else
        {
          screen11();
        }

        break;
      }

      case 2: // Back
      {
        screen7();
        break;
      }
      }
      break;
    }

    case 10: // Low voltage cut-off adjustment screen
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
        screen9();
        selectedDigit = 0;
      }
      }
      break;
    }

    case 11: // Current monitor screen
    {
      screen18();
      break;
    }

    case 12: // Power monitor screen
    {
      screen18();
      break;
    }

    case 13: // No/invaid SD card screen
    {
      screen6();
      break;
    }

    case 14: // Overtemperature screen
    {
      screen16();
      break;
    }

    case 15: // Low-voltage cut-off termination screen
    {
      screen16();
      break;
    }

    case 16: // Analysis complete screen
    {
      screen1();
      break;
    }

    case 17: // Analysis terminated screen
    {
      screen1();
      break;
    }
    
    case 18: // Confirm terminate analysis screen
    {

      switch (cursorPosition)
      {
      case 0: // Yes
      {
        screen17();

        // Terminate Analaysis
        controller.set_enable(false);

        break;
      }
      case 1: // No
      {
        if (powerMode)
        {
          screen12();
        }
        else
        {
          screen11();
        }
        break;
      }
      }
      break;
    }
    }
  }
}
