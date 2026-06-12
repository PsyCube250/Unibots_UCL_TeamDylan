#include "AlashMotorControlLite.h"

// Motor definitions (PWM_PWM mode) - 4 motors
AlashMotorControlLite motor1(PWM_PWM, PB4, PB5);
AlashMotorControlLite motor2(PWM_PWM, PB6, PB7);
AlashMotorControlLite motor3(PWM_PWM, PB8, PB9);
AlashMotorControlLite motor4(PWM_PWM, PA0, PA1);

// Serial protocol:
// Jetson sends 4 bytes: [speed1, speed2, speed3, speed4]
// Each byte is signed (-100 to 100), transmitted as uint8 offset by 100
// So wire value 0 = speed -100, wire value 100 = speed 0, wire value 200 = speed 100
// Frame format: [0xFF] [0xFE] [m1] [m2] [m3] [m4] [checksum]
// Checksum = (m1 + m2 + m3 + m4) & 0xFF

// USART2: PA2=TX, PA3=RX for Jetson UART connection
// This keeps Serial (PA9/PA10/USB) free for debugging
HardwareSerial jetsonSerial(PA3, PA2);  // (RX, TX)

#define HEADER1 0xFF
#define HEADER2 0xFE
#define FRAME_SIZE 7
#define TIMEOUT_MS 500

uint8_t buf[FRAME_SIZE];
uint8_t buf_idx = 0;
unsigned long last_msg_time = 0;

void setMotors(int s1, int s2, int s3, int s4) {
  motor1.setSpeed(s1);
  motor2.setSpeed(s2);
  motor3.setSpeed(s3);
  motor4.setSpeed(s4);
}

void stopMotors() {
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
}

void setup() {
  jetsonSerial.begin(115200);
  stopMotors();
}

void loop() {
  while (jetsonSerial.available()) {
    uint8_t b = jetsonSerial.read();

    if (buf_idx == 0) {
      if (b == HEADER1) buf[buf_idx++] = b;
    } else if (buf_idx == 1) {
      if (b == HEADER2) buf[buf_idx++] = b;
      else buf_idx = 0;
    } else {
      buf[buf_idx++] = b;

      if (buf_idx == FRAME_SIZE) {
        uint8_t checksum = (buf[2] + buf[3] + buf[4] + buf[5]) & 0xFF;

        if (checksum == buf[6]) {
          int speed1 = constrain((int)buf[2] - 100, -100, 100);
          int speed2 = constrain((int)buf[3] - 100, -100, 100);
          int speed3 = constrain((int)buf[4] - 100, -100, 100);
          int speed4 = constrain((int)buf[5] - 100, -100, 100);

          setMotors(speed1, speed2, speed3, speed4);
          last_msg_time = millis();
        }

        buf_idx = 0;
      }
    }
  }

  // Safety: stop motors if no message received within timeout
  if (millis() - last_msg_time > TIMEOUT_MS) {
    stopMotors();
  }
}
