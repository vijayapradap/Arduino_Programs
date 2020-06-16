#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Wire.requestFrom(8,1); 
  Wire.beginTransmission(8);
  Wire.write('A');
  Wire.endTransmission();
  delay(2000);
}
