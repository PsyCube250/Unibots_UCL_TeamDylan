#include "AlashMotorControlLite.h"

// --- Drive Motors ---
AlashMotorControlLite motorFL(PWM_PWM, PA0, PA1); // Front-Left
AlashMotorControlLite motorFR(PWM_PWM, PA2, PA3); // Front-Right
AlashMotorControlLite motorBL(PWM_PWM, PA6, PA7); // Back-Left
AlashMotorControlLite motorBR(PWM_PWM, PB0, PB1); // Back-Right

// --- Dropoff Slider Motors (activated at end of match against wall) ---
AlashMotorControlLite motorDROP1(PWM_PWM, PB6, PB7);
AlashMotorControlLite motorDROP2(PWM_PWM, PB8, PB9);

String inputString = "";

const int TURN_BASE_SPEED    = 65;
const int COLLECT_CRAWL_SPEED = 40;
const int DROP_SPEED          = 100;
const unsigned long DROP_DURATION_MS = 20000;

// Non-blocking drop state
bool dropping = false;
unsigned long dropStartTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("STM32 READY");
  inputString.reserve(32);
  stopAll();

  pinMode(PB3, OUTPUT);
  digitalWrite(PB3,HIGH);
}

void loop() {
  // Non-blocking drop timer
  if (dropping && (millis() - dropStartTime >= DROP_DURATION_MS)) {
    dropping = false;
    motorDROP1.stop();
    motorDROP2.stop();
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
  String action   = (separatorIndex > 0) ? cmd.substring(0, separatorIndex) : cmd;
  action="FORWARD";
  String valueStr = (separatorIndex > 0) ? cmd.substring(separatorIndex + 1) : "";

  if (action == "FORWARD") {
    Serial.println("FORWARD");
    int speed = valueStr.toInt();
    setMecanum(speed, speed, speed, speed);
  }
  else if (action == "TURN") {
    Serial.println("TURN");
    int angle = valueStr.toInt();
    int speed = (angle > 0) ? TURN_BASE_SPEED : -TURN_BASE_SPEED;
    // Tank turn: left wheels fwd, right wheels back
    setMecanum(speed, -speed, speed, -speed);
  }
  else if (action == "STOP") {
    dropping = false;
    Serial.println("STOP");
    stopAll();
  }
  else if (action == "COLLECT") {
    Serial.println("COLLECT");
    // Crawl forward slowly to collect ball
    setMecanum(COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED,
               COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED);
  }
  else if (action == "DROP") {
    Serial.println("DROP");
    // Stop drive motors, run sliders for 20 seconds non-blocking
    setMecanum(0, 0, 0, 0);
    motorDROP1.setSpeed(DROP_SPEED);
    motorDROP2.setSpeed(DROP_SPEED);
    dropping = true;
    dropStartTime = millis();
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
  motorDROP1.stop();
  motorDROP2.stop();
}
