void setup(){
  pinMode(PC13, OUTPUT);
}

void loop(){
  digitalWrite(PC13, HIGH);
  delay(300);
  digitalWrite(PC13, LOW);
  delay(300);
}