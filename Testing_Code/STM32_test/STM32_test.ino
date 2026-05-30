void setup() {
  Serial.begin(115200);
  delay(500);             // ← give Serial time to stabilise
  pinMode(PB3, OUTPUT);
  Serial.println("BOOT COMPLETE");
}

void loop() {
  digitalWrite(PB3, HIGH);
  Serial.println("HIGH");
  delay(300);
  digitalWrite(PB3, LOW);
  Serial.println("LOW");
  delay(300);
}