#include <Arduino.h>

void setup()
{
  pinMode(7, OUTPUT);
}

void loop()
{
  digitalWrite(7, HIGH);
  delay(3);
  digitalWrite(7, LOW);
  delay(3);
}