void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0;i<8;i++) {
    digitalWrite(i,HIGH);
    delay(1000);
  }
  for(int i=8;i>0;i--) {
    digitalWrite(i,LOW);
    delay(1000);
  }
}
