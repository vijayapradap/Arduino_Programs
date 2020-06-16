#include <SPI.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(0,1,2,3,4,5);
int SS1=9;
char buff[50];
int a=0;

void setup() {
  // put your setup code here, to run once:
  pinMode(SS, OUTPUT);
  pinMode(SS1, OUTPUT);
  digitalWrite(SS, HIGH);
  digitalWrite(SS1, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}
void spi_send(char *data) {
  digitalWrite(SS, LOW);
  
  while(*data!='\0') {
    SPI.transfer (*data++);
  }
  
  digitalWrite(SS, HIGH);
}
void spi_send1(char *data) {
  digitalWrite(SS1, LOW);
  
  while(*data!='\0') {
    SPI.transfer (*data++);
  }
  
  digitalWrite(SS1, HIGH);
}
void loop() {
  // put your main code here, to run repeatedly:
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("THIS IS MASTER");
  delay(1000);
  spi_send("DATA TO SLAVE1");
  delay(1000);
  spi_send1("DATA TO SLAVE2");
  delay(1000);
}
