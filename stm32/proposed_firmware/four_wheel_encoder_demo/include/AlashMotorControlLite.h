#pragma once

#include <Arduino.h>

// Minimal local compatibility layer for the subset of AlashMotorControlLite
// used by the encoder demo. It drives DRV8833/TB6612-style IN1/IN2 inputs in
// PWM_PWM mode with speed values in -100..100.
static const uint8_t PWM_PWM = 1;

class AlashMotorControlLite {
public:
  AlashMotorControlLite(uint8_t mode, uint8_t pinA, uint8_t pinB)
      : mode_(mode), pinA_(pinA), pinB_(pinB), configured_(false) {}

  void setSpeed(int speed) {
    configurePins();

    speed = constrain(speed, -100, 100);
    int duty = map(abs(speed), 0, 100, 0, 255);

    if (mode_ != PWM_PWM) {
      stop();
      return;
    }

    if (speed > 0) {
      analogWrite(pinA_, duty);
      analogWrite(pinB_, 0);
    } else if (speed < 0) {
      analogWrite(pinA_, 0);
      analogWrite(pinB_, duty);
    } else {
      stop();
    }
  }

  void stop() {
    configurePins();
    analogWrite(pinA_, 0);
    analogWrite(pinB_, 0);
  }

  void brake() {
    configurePins();
    analogWrite(pinA_, 255);
    analogWrite(pinB_, 255);
  }

private:
  void configurePins() {
    if (configured_) return;
    pinMode(pinA_, OUTPUT);
    pinMode(pinB_, OUTPUT);
    analogWrite(pinA_, 0);
    analogWrite(pinB_, 0);
    configured_ = true;
  }

  uint8_t mode_;
  uint8_t pinA_;
  uint8_t pinB_;
  bool configured_;
};
