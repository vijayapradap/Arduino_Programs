#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(0,1,2,3,4,5);

char buff[50];
int a=0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  a=0;
  Wire.beginTransmission(8);
  Wire.write("VIJAYAPRADAP M");
  Wire.endTransmission();
  delay(2000);
  Wire.requestFrom(8,1);
  //while(Wire.available()) {
  //  buff[a]=Wire.read();
  //  a++;
  //}
  char byte = Wire.read();
  delay(500);
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print(byte);
  delay(2000);
}
