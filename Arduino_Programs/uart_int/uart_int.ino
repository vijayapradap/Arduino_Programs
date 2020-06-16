#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,2,3,4,5);
String inputString = "";
bool stringComplete = false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  inputString.reserve(200);
}
void loop() {
  // put your main code here, to run repeatedly:
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UART TESTING...");
  lcd.setCursor(0, 1);
  if (stringComplete == true) {
    lcd.print(inputString);    
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
  }
  delay(2000);
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char) Serial.read();
    inputString += inChar;
    if (inChar == 0x0d)
      stringComplete = true;
  }
}
