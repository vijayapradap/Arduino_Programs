void setup() {
  // put your setup code here, to run once:
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(0) == 1) {
    digitalWrite(1, HIGH);
  } else {
    digitalWrite(1, LOW);
  }
}
