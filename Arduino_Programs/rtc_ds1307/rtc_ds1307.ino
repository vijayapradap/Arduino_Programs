#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(0,1,2,3,4,5);
int Sec,Min,Hour,Hr,Date,Month,Year,ap;

byte bcd2bin(byte val) {
  return( (val/10*16) + (val%10) );
}
byte bin2bcd(byte val) {
  return val + 6 * (val / 10);
}
void RTC_Write(int sec, int min, int hour, int day, int month, int year) {
  /*char I2CData[] = {0x00, 0x00, 0x52, 0X01, 0x01, 0x01, 0x18, 0x00, 0x00};
  Wire.beginTransmission(0x68);
  Wire.write(byte(0x00));
  for(int i=0;i<8;i++) {
    Wire.write(I2CData[i]);
  }
  Wire.endTransmission();*/
  
  Wire.beginTransmission(0x68);
  Wire.write(byte(0x00));
  Wire.write(bin2bcd(sec));
  Wire.write(bin2bcd(min));
  Wire.write(bin2bcd(hour));
  Wire.write(bin2bcd(0));
  Wire.write(bin2bcd(day));
  Wire.write(bin2bcd(month));
  Wire.write(bin2bcd(year));
  Wire.endTransmission();
}
void bcd_ascii(char value) {
    char temp1, temp2;
    temp1=value;
    temp1=temp1&0xf0;
    temp1=temp1>>4;
    temp1=temp1|0x30;
    lcd.print(temp1);
    temp2=value;
    temp2=temp2&0x0f;
    temp2=temp2|0x30;
    lcd.print(temp2);
}
void RTC_Read() {
  Wire.beginTransmission(0x68);
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Wire.requestFrom(0x68, 7);
  
  Sec = Wire.read();
  Min = Wire.read();
  Hour = Wire.read();
  Hr=Hour & 0x1F;
  ap=Hour & 0x20;
  byte dayOfWeek = Wire.read();
  Date = Wire.read();
  Month = Wire.read();
  Year = Wire.read();
  
  lcd.print("TIME:");
  bcd_ascii(Hr);
  lcd.print(':');
  bcd_ascii(Min);
  lcd.print(':');
  bcd_ascii(Sec);
  lcd.setCursor(0,1);
  lcd.print("DATE:");
  bcd_ascii(Date);
  lcd.print('/');
  bcd_ascii(Month);
  lcd.print('/');
  bcd_ascii(Year);
}
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  RTC_Write(00, 10, 8, 29, 03, 19);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  while(1) {
    lcd.clear();
    RTC_Read();
    delay(1000);
  }
}
