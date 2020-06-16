#include <EEPROM.h>
char buff[30];

void setup() {
  Serial.begin(9600);
}
void loop() {
  char name[]="VIJAYARPADAP M";
  
  Serial.println("Writing in EEPROM...");
  for (int i = 15, j=0; name[j]!='\0'; i++, j++) {
    EEPROM.write(i, name[j]);
  }
  Serial.println();
  Serial.println("Reading in EEPROM...");
  for (int a=0, b=0; a<30; a++, b++) {
    buff[b] = EEPROM.read(a);
  }
  Serial.println(buff);
  while(1);
}
