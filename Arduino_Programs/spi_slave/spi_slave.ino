#include <SPI.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,2,3,4,5);
char buff[50];
int a=0;

void setup() {
  // put your setup code here, to run once:
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
}
void loop() {
  // put your main code here, to run repeatedly:
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("THIS IS SLAVE1");
  lcd.setCursor(0,1);
  lcd.print(buff);
  delay(1000);
}
ISR (SPI_STC_vect) {
  buff[a++]=SPDR;
}
