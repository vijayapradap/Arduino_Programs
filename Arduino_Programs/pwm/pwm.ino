#include <LiquidCrystal.h>
LiquidCrystal lcd(0,1,2,3,4,5);
int ledPin = 6;
int analogPin = A0;
int val = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  val = analogRead(analogPin);
  val = map(val, 0, 1023, 0, 255);
  analogWrite(ledPin, val);
  lcd.print(val);
  lcd.setCursor(0,1);
  lcd.print(val*5/255);
  delay(100);
}
