#include "AlashMotorControlLite.h"

// --- Motor Definitions ---
AlashMotorControlLite motorFL(PWM_PWM, PA0, PA1);
AlashMotorControlLite motorFR(PWM_PWM, PA2, PA3);
AlashMotorControlLite motorBL(PWM_PWM, PA6, PA7);
AlashMotorControlLite motorBR(PWM_PWM, PB0, PB1);

AlashMotorControlLite motorDROPOFF1(PWM_PWM, PB4, PB5);
AlashMotorControlLite motorDROPOFF2(PWM_PWM, PB6, PB7);

String inputString = "";

const int TURN_BASE_SPEED   = 65;
const int COLLECT_CRAWL_SPEED = 40;
const int DROP_SPEED        = 100;
const unsigned long DROP_DURATION_MS = 20000;

// Non-blocking dropoff state
bool dropping = false;
unsigned long dropStartTime = 0;

void setup() {
  Serial.begin(115200);
  inputString.reserve(32);
  stopAll();
}

void loop() {
  // Non-blocking dropoff timer
  if (dropping && (millis() - dropStartTime >= DROP_DURATION_MS)) {
    dropping = false;
    stopAll();
    Serial.println("DROP_DONE");
  }

  // Non-blocking serial read
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseCommand(inputString);
      inputString = "";
    } else if (inChar != '\r') {
      inputString += inChar;
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();

  int separatorIndex = cmd.indexOf(',');
  String action = (separatorIndex > 0) ? cmd.substring(0, separatorIndex) : cmd;
  String valueStr = (separatorIndex > 0) ? cmd.substring(separatorIndex + 1) : "";

  if (action == "FORWARD") {
    int speed = valueStr.toInt();
    setMecanum(speed, speed, speed, speed);
  }
  else if (action == "TURN") {
    int angle = valueStr.toInt();
    // Positive = right, Negative = left
    int speed = (angle > 0) ? TURN_BASE_SPEED : -TURN_BASE_SPEED;
    // Tank turn: left wheels fwd, right wheels back
    setMecanum(speed, -speed, speed, -speed);
  }
  else if (action == "STOP") {
    Serial.println('STOP');
    dropping = false;
    stopAll();
  }
  else if (action == "COLLECT") {
    // Crawl forward slowly to collect ball
    setMecanum(COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED,
               COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED);
  }
  else if (action == "DROP") {
    // Non-blocking dropoff — runs for DROP_DURATION_MS then auto-stops
    motorDROPOFF1.setSpeed(DROP_SPEED);
    motorDROPOFF2.setSpeed(DROP_SPEED);
    dropping = true;
    dropStartTime = millis();
    stopAll(); // Stop drive motors while dropping
  }
}

void setMecanum(int fl, int fr, int bl, int br) {
  motorFL.setSpeed(fl);
  motorFR.setSpeed(fr);
  motorBL.setSpeed(bl);
  motorBR.setSpeed(br);
}

void stopAll() {
  motorFL.stop();
  motorFR.stop();
  motorBL.stop();
  motorBR.stop();
  motorDROPOFF1.stop();
  motorDROPOFF2.stop();
}