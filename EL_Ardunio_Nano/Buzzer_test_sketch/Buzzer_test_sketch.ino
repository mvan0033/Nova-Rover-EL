#define PIN 7

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN, LOW);
  delayMicroseconds(500);
  digitalWrite(PIN, HIGH);
  delayMicroseconds(500);
}
