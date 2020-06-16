#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(0,1,2,3,4,5);

char buff[50];
int a=0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  a=0;
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print(buff);
  delay(1000);
}

void receiveEvent(int howMany) {
  while(Wire.available()) {
    buff[a++]=Wire.read();
  }
}
void requestEvent() {
  Wire.write('A');
}
