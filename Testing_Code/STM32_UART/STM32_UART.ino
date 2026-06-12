#include "AlashMotorControlLite.h"

// --- Hardware Pin Definitions (REPLACE WITH YOUR ACTUAL PINS) ---
AlashMotorControlLite motorFL(PWM_PWM, PA0, PA1); // Front-Left
AlashMotorControlLite motorFR(PWM_PWM, PA2, PA3); // Front-Right
AlashMotorControlLite motorBL(PWM_PWM, PA6, PA7); // Back-Left
AlashMotorControlLite motorBR(PWM_PWM, PB0, PB1); // Back-Right

// Metal Dropping Mechanism (Iroller)
AlashMotorControlLite motorPICKUP1(PWM_PWM, PB4, PB5); 
AlashMotorControlLite motorPICKUP2(PWM_PWM, PB6, PB7); 

String inputString = "";

// Tuning speeds for non-variable actions
const int TURN_BASE_SPEED = 65;    
const int COLLECT_CRAWL_SPEED = 40; 
const int BEARING_PICKUP_SPEED = 100; 


void setup() {
  Serial.begin(115200);
  inputString.reserve(32); // Prevent heap fragmentation
  stopAll();
}

void loop() {
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
  String action = cmd;
  String valueStr = "";

  if (separatorIndex > 0) {
    action = cmd.substring(0, separatorIndex);
    valueStr = cmd.substring(separatorIndex + 1);
  }

  // --- Execution Logic ---
  if (action == "FORWARD") {
    int speed = valueStr.toInt();
    setMecanum(speed, speed, speed, speed);
  }
  else if (action == "TURN") {
    int angle = valueStr.toInt();
    // Positive angle = Turn Right, Negative = Turn Left
    int speed = (angle > 0) ? TURN_BASE_SPEED : -TURN_BASE_SPEED;
    
    // Tank Turn: Left wheels forward, Right wheels backward
    setMecanum(speed, -speed, speed, -speed);
  }
  else if (action == "STOP") {
    motorPICKUP1.setSpeed(BEARING_PICKUP_SPEED);
    motorPICKUP2.setSpeed(BEARING_PICKUP_SPEED);
    unsigned long time = millis();
    while (millis()-time<=20000){
    }
    stopAll();
  }
  else if (action == "COLLECT") {
    // Jetson sends COLLECT once. Turn on intake and crawl forward over the ball.
    setMecanum(COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED, COLLECT_CRAWL_SPEED);
  }
}

// Drive kinematics mapping
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
  motorPICKUP1.stop();
  motorPICKUP2.stop();
}