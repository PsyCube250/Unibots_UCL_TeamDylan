#include <Wire.h>
#include <Motoron.h>

// Top Shield (0x14 / 20) -> Front Wheels
MotoronI2C mcFront(20);

// Bottom Shield (0x10 / 16) -> Back Wheels
MotoronI2C mcBack(16);

// Motor port mapping (Assuming you plugged Left into M1 and Right into M2)
const int PORT_LEFT = 1;
const int PORT_RIGHT = 2;

void setup() {
  Wire.begin();
  
  // Initialize Front Shield
  mcFront.reinitialize();
  mcFront.clearResetFlag();
  mcFront.disableCommandTimeout();

  // Initialize Back Shield
  mcBack.reinitialize();
  mcBack.clearResetFlag();
  mcBack.disableCommandTimeout();
}

void loop() {
  // Test 1: All wheels forward at half speed (Max Motoron speed is 800)
  setMecanum(400, 400, 400, 400);
  delay(2000);

  // Stop
  setMecanum(0, 0, 0, 0);
  delay(1000);

  // Test 2: All wheels reverse
  setMecanum(-400, -400, -400, -400);
  delay(2000);

  // Stop
  setMecanum(0, 0, 0, 0);
  delay(2000);
}

// Helper function to map speeds to the correct I2C shield and port
void setMecanum(int fl, int fr, int bl, int br) {
  // Front wheels -> Top Shield
  mcFront.setSpeed(PORT_LEFT, fl);
  mcFront.setSpeed(PORT_RIGHT, fr);
  
  // Back wheels -> Bottom Shield
  mcBack.setSpeed(PORT_LEFT, bl);
  mcBack.setSpeed(PORT_RIGHT, br);
}