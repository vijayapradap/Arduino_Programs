#include <LiquidCrystal.h>
LiquidCrystal lcd(0,1,2,3,4,5);
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.begin(16, 2);
  lcd.setCursor(0,1);
  lcd.print("HELLO WORLD!");
  delay(2000);
}
