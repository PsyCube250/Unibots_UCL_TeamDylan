#include "AlashMotorControlLite.h"

// Jetson Nano UART link on STM32 USART1.
// STM32 PA9  TX -> Jetson Pin 10 RX
// STM32 PA10 RX <- Jetson Pin 8 TX
#define CommandSerial Serial1

/*
  Four-wheel mecanum/omni encoder demo for STM32Duino.

  Safety defaults:
  - Motors are disabled at boot.
  - PWM is clamped to +/-15 by default.
  - Motion commands time out after 500 ms.
  - Encoder telemetry is printed even when motor output is disabled.

  Serial commands at 115200 baud, newline terminated:
    HELP
    PING
    STATUS
    STOP
    ENABLE 0
    ENABLE 1
    LIMIT 15
    MOTOR_SIGN fl fr bl br values must be -1 or 1
    ENC_SIGN fl fr bl br   values must be -1 or 1
    ZERO_ENC
    PULSE wheel pwm ms     wheel is 0..3 or FL/FR/BL/BR; requires ENABLE 1
    PWM fl fr bl br       values in -100..100, clamped by LIMIT
    VEL fl fr bl br       target encoder ticks/second
    TELEM hz              1..50

  Telemetry format:
    T seq ms mode enabled limit enc0 enc1 enc2 enc3 tps0 tps1 tps2 tps3 pwm0 pwm1 pwm2 pwm3 target0 target1 target2 target3 fault
*/

// ---- Motor pin map: DRV8833-style IN1/IN2 per wheel.
// Matches the currently occupied motor-shield pins reported by the team.
AlashMotorControlLite motorFL(PWM_PWM, PA0, PA1);
AlashMotorControlLite motorFR(PWM_PWM, PA2, PA3);
AlashMotorControlLite motorBL(PWM_PWM, PA6, PA7);
AlashMotorControlLite motorBR(PWM_PWM, PB0, PB1);

AlashMotorControlLite* motors[4] = {&motorFL, &motorFR, &motorBL, &motorBR};

// Existing STM32_UART.ino drove PB3 high; keep it as a shield-enable style pin.
const uint8_t SHIELD_ENABLE_PIN = PB3;

// ---- Encoder pin map: quadrature A/B per wheel.
// Uses interrupts for the first safe demo. For high RPM production control,
// move to hardware timer encoder mode after the exact STM32 MCU is confirmed.
// Avoids occupied pins:
//   PA0 PA1 PA2 PA3 PA6 PA7 PB0 PB1 PB3 PB6 PB7 PB8 PB9 = motor shield
//   PA9 PA10 = Jetson UART
const uint8_t ENC_A[4] = {PB10, PB12, PB14, PA4};
const uint8_t ENC_B[4] = {PB11, PB13, PB15, PA5};

// Runtime-adjustable signs avoid reflashing during bring-up.
// Use -1 if a wheel/encoder is reversed.
int8_t encoderSign[4] = {1, 1, 1, 1};
int8_t motorSign[4] = {1, 1, 1, 1};

const uint32_t BAUD_RATE = 115200;
const uint8_t STATUS_LED_PIN = PC13;
const uint16_t WATCHDOG_MS = 500;
const uint16_t CONTROL_PERIOD_MS = 50;
const uint16_t DEFAULT_TELEM_PERIOD_MS = 100;
const int HARD_PWM_LIMIT = 45;
const int STARTUP_PWM_LIMIT = 15;

enum ControlMode : uint8_t {
  MODE_STOP = 0,
  MODE_PWM = 1,
  MODE_VELOCITY = 2
};

volatile long encoderCounts[4] = {0, 0, 0, 0};
volatile uint8_t encoderState[4] = {0, 0, 0, 0};

long lastCounts[4] = {0, 0, 0, 0};
float measuredTicksPerSec[4] = {0, 0, 0, 0};
float targetTicksPerSec[4] = {0, 0, 0, 0};
int targetPwm[4] = {0, 0, 0, 0};
int appliedPwm[4] = {0, 0, 0, 0};
float integral[4] = {0, 0, 0, 0};

ControlMode mode = MODE_STOP;
bool motorsEnabled = false;
int pwmLimit = STARTUP_PWM_LIMIT;
uint32_t lastMotionCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemMs = 0;
uint16_t telemPeriodMs = DEFAULT_TELEM_PERIOD_MS;
uint32_t seq = 0;
uint16_t faultFlags = 0;
bool heartbeatLedOn = false;
uint32_t lastHeartbeatMs = 0;
uint32_t pulseStopMs = 0;

char lineBuf[96];
uint8_t lineLen = 0;

const int8_t QUAD_TABLE[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

void updateEncoder(uint8_t i) {
  uint8_t a = digitalRead(ENC_A[i]) ? 1 : 0;
  uint8_t b = digitalRead(ENC_B[i]) ? 1 : 0;
  uint8_t next = (a << 1) | b;
  uint8_t idx = (encoderState[i] << 2) | next;
  encoderCounts[i] += QUAD_TABLE[idx] * encoderSign[i];
  encoderState[i] = next;
}

void isrEnc0A() { updateEncoder(0); }
void isrEnc0B() { updateEncoder(0); }
void isrEnc1A() { updateEncoder(1); }
void isrEnc1B() { updateEncoder(1); }
void isrEnc2A() { updateEncoder(2); }
void isrEnc2B() { updateEncoder(2); }
void isrEnc3A() { updateEncoder(3); }
void isrEnc3B() { updateEncoder(3); }

void stopAllMotors() {
  for (uint8_t i = 0; i < 4; ++i) {
    motors[i]->stop();
    appliedPwm[i] = 0;
    targetPwm[i] = 0;
    targetTicksPerSec[i] = 0.0f;
    integral[i] = 0.0f;
  }
  mode = MODE_STOP;
  pulseStopMs = 0;
}

int clampPwm(int value) {
  if (value > pwmLimit) return pwmLimit;
  if (value < -pwmLimit) return -pwmLimit;
  return value;
}

void applyMotor(uint8_t i, int pwm) {
  pwm = clampPwm(pwm) * motorSign[i];
  appliedPwm[i] = pwm;

  if (!motorsEnabled || mode == MODE_STOP) {
    motors[i]->stop();
    appliedPwm[i] = 0;
    return;
  }

  if (pwm == 0) {
    motors[i]->stop();
  } else {
    motors[i]->setSpeed(pwm);
  }
}

void printHelp() {
  CommandSerial.println("OK commands: HELP PING STATUS STOP ENABLE 0|1 LIMIT n MOTOR_SIGN a b c d ENC_SIGN a b c d ZERO_ENC PULSE wheel pwm ms PWM fl fr bl br VEL fl fr bl br TELEM hz");
}

bool parseFourInts(char* token, int out[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    token = strtok(nullptr, " ,\t");
    if (!token) return false;
    out[i] = atoi(token);
  }
  return true;
}

bool parseFourSigns(int8_t out[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    char* token = strtok(nullptr, " ,\t");
    if (!token) return false;
    int value = atoi(token);
    if (value != -1 && value != 1) return false;
    out[i] = (int8_t)value;
  }
  return true;
}

int parseWheelIndex(char* token) {
  if (!token) return -1;
  if (!strcmp(token, "0") || !strcmp(token, "FL")) return 0;
  if (!strcmp(token, "1") || !strcmp(token, "FR")) return 1;
  if (!strcmp(token, "2") || !strcmp(token, "BL")) return 2;
  if (!strcmp(token, "3") || !strcmp(token, "BR")) return 3;
  return -1;
}

void zeroEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) {
    encoderCounts[i] = 0;
  }
  interrupts();
  for (uint8_t i = 0; i < 4; ++i) {
    lastCounts[i] = 0;
    measuredTicksPerSec[i] = 0.0f;
  }
}

void printStatus() {
  CommandSerial.println("OK STATUS version=bringup_20260623 uart=Serial1 rx=PA10 tx=PA9 shield=PB3 led=PC13");
  CommandSerial.println("OK MOTORS FL=PA0,PA1 FR=PA2,PA3 BL=PA6,PA7 BR=PB0,PB1");
  CommandSerial.println("OK ENCODERS FL=PB10,PB11 FR=PB12,PB13 BL=PB14,PB15 BR=PA4,PA5");
  CommandSerial.print("OK MOTOR_SIGN ");
  for (uint8_t i = 0; i < 4; ++i) {
    if (i) CommandSerial.print(' ');
    CommandSerial.print(motorSign[i]);
  }
  CommandSerial.println();
  CommandSerial.print("OK ENC_SIGN ");
  for (uint8_t i = 0; i < 4; ++i) {
    if (i) CommandSerial.print(' ');
    CommandSerial.print(encoderSign[i]);
  }
  CommandSerial.println();
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " ,\t");
  if (!cmd) return;

  if (!strcmp(cmd, "HELP")) {
    printHelp();
    return;
  }

  if (!strcmp(cmd, "PING")) {
    CommandSerial.println("OK PONG");
    return;
  }

  if (!strcmp(cmd, "STATUS")) {
    printStatus();
    return;
  }

  if (!strcmp(cmd, "STOP")) {
    stopAllMotors();
    faultFlags = 0;
    CommandSerial.println("OK STOP");
    return;
  }

  if (!strcmp(cmd, "ENABLE")) {
    char* arg = strtok(nullptr, " ,\t");
    motorsEnabled = arg && atoi(arg) == 1;
    if (!motorsEnabled) {
      stopAllMotors();
    }
    CommandSerial.print("OK ENABLE ");
    CommandSerial.println(motorsEnabled ? 1 : 0);
    return;
  }

  if (!strcmp(cmd, "LIMIT")) {
    char* arg = strtok(nullptr, " ,\t");
    if (!arg) {
      CommandSerial.println("ERR LIMIT missing");
      return;
    }
    pwmLimit = constrain(atoi(arg), 0, HARD_PWM_LIMIT);
    CommandSerial.print("OK LIMIT ");
    CommandSerial.println(pwmLimit);
    return;
  }

  if (!strcmp(cmd, "TELEM")) {
    char* arg = strtok(nullptr, " ,\t");
    if (!arg) {
      CommandSerial.println("ERR TELEM missing");
      return;
    }
    int hz = constrain(atoi(arg), 1, 50);
    telemPeriodMs = 1000 / hz;
    CommandSerial.print("OK TELEM ");
    CommandSerial.println(hz);
    return;
  }

  if (!strcmp(cmd, "MOTOR_SIGN")) {
    int8_t values[4];
    if (!parseFourSigns(values)) {
      CommandSerial.println("ERR MOTOR_SIGN needs four values, each -1 or 1");
      return;
    }
    stopAllMotors();
    for (uint8_t i = 0; i < 4; ++i) motorSign[i] = values[i];
    CommandSerial.println("OK MOTOR_SIGN");
    return;
  }

  if (!strcmp(cmd, "ENC_SIGN")) {
    int8_t values[4];
    if (!parseFourSigns(values)) {
      CommandSerial.println("ERR ENC_SIGN needs four values, each -1 or 1");
      return;
    }
    for (uint8_t i = 0; i < 4; ++i) encoderSign[i] = values[i];
    zeroEncoders();
    CommandSerial.println("OK ENC_SIGN");
    return;
  }

  if (!strcmp(cmd, "ZERO_ENC")) {
    zeroEncoders();
    CommandSerial.println("OK ZERO_ENC");
    return;
  }

  if (!strcmp(cmd, "PULSE")) {
    char* wheelToken = strtok(nullptr, " ,\t");
    char* pwmToken = strtok(nullptr, " ,\t");
    char* msToken = strtok(nullptr, " ,\t");
    int wheel = parseWheelIndex(wheelToken);
    if (wheel < 0 || !pwmToken || !msToken) {
      CommandSerial.println("ERR PULSE needs wheel pwm ms");
      return;
    }
    if (!motorsEnabled) {
      CommandSerial.println("ERR PULSE requires ENABLE 1");
      return;
    }
    int pwm = constrain(atoi(pwmToken), -pwmLimit, pwmLimit);
    int durationMs = constrain(atoi(msToken), 50, 400);
    for (uint8_t i = 0; i < 4; ++i) {
      targetPwm[i] = 0;
      integral[i] = 0.0f;
    }
    targetPwm[wheel] = pwm;
    mode = MODE_PWM;
    lastMotionCommandMs = millis();
    pulseStopMs = millis() + (uint32_t)durationMs;
    CommandSerial.println("OK PULSE");
    return;
  }

  if (!strcmp(cmd, "PWM")) {
    int values[4];
    if (!parseFourInts(cmd, values)) {
      CommandSerial.println("ERR PWM needs 4 values");
      return;
    }
    for (uint8_t i = 0; i < 4; ++i) {
      targetPwm[i] = constrain(values[i], -100, 100);
      integral[i] = 0.0f;
    }
    mode = MODE_PWM;
    lastMotionCommandMs = millis();
    CommandSerial.println("OK PWM");
    return;
  }

  if (!strcmp(cmd, "VEL")) {
    int values[4];
    if (!parseFourInts(cmd, values)) {
      CommandSerial.println("ERR VEL needs 4 values");
      return;
    }
    for (uint8_t i = 0; i < 4; ++i) {
      targetTicksPerSec[i] = (float)values[i];
      integral[i] = 0.0f;
    }
    mode = MODE_VELOCITY;
    lastMotionCommandMs = millis();
    CommandSerial.println("OK VEL");
    return;
  }

  CommandSerial.println("ERR unknown command");
}

void readSerialLines() {
  while (CommandSerial.available()) {
    char c = (char)CommandSerial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
      CommandSerial.println("ERR line too long");
    }
  }
}

void updateControl() {
  uint32_t now = millis();
  uint32_t dtMs = now - lastControlMs;
  if (dtMs < CONTROL_PERIOD_MS) return;
  lastControlMs = now;

  if (mode != MODE_STOP && now - lastMotionCommandMs > WATCHDOG_MS) {
    faultFlags |= 0x0001;
    stopAllMotors();
  }

  if (pulseStopMs && (int32_t)(now - pulseStopMs) >= 0) {
    stopAllMotors();
  }

  long counts[4];
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) counts[i] = encoderCounts[i];
  interrupts();

  float dt = dtMs / 1000.0f;
  for (uint8_t i = 0; i < 4; ++i) {
    long delta = counts[i] - lastCounts[i];
    lastCounts[i] = counts[i];
    measuredTicksPerSec[i] = delta / dt;
  }

  if (mode == MODE_PWM) {
    for (uint8_t i = 0; i < 4; ++i) {
      applyMotor(i, targetPwm[i]);
    }
  } else if (mode == MODE_VELOCITY) {
    // Conservative starter PI. Tune only during lifted-wheel tests.
    const float kp = 0.08f;
    const float ki = 0.025f;
    for (uint8_t i = 0; i < 4; ++i) {
      float err = targetTicksPerSec[i] - measuredTicksPerSec[i];
      integral[i] += err * dt;
      integral[i] = constrain(integral[i], -300.0f, 300.0f);
      int pwm = (int)(kp * err + ki * integral[i]);
      applyMotor(i, pwm);
    }
  } else {
    for (uint8_t i = 0; i < 4; ++i) {
      applyMotor(i, 0);
    }
  }
}

void printTelemetry() {
  uint32_t now = millis();
  if (now - lastTelemMs < telemPeriodMs) return;
  lastTelemMs = now;

  long counts[4];
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) counts[i] = encoderCounts[i];
  interrupts();

  CommandSerial.print("T ");
  CommandSerial.print(seq++);
  CommandSerial.print(' ');
  CommandSerial.print(now);
  CommandSerial.print(' ');
  CommandSerial.print((int)mode);
  CommandSerial.print(' ');
  CommandSerial.print(motorsEnabled ? 1 : 0);
  CommandSerial.print(' ');
  CommandSerial.print(pwmLimit);

  for (uint8_t i = 0; i < 4; ++i) {
    CommandSerial.print(' ');
    CommandSerial.print(counts[i]);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    CommandSerial.print(' ');
    CommandSerial.print(measuredTicksPerSec[i], 1);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    CommandSerial.print(' ');
    CommandSerial.print(appliedPwm[i]);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    CommandSerial.print(' ');
    if (mode == MODE_VELOCITY) {
      CommandSerial.print(targetTicksPerSec[i], 1);
    } else {
      CommandSerial.print(targetPwm[i]);
    }
  }
  CommandSerial.print(' ');
  CommandSerial.println(faultFlags);
}

void setupEncoders() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    uint8_t a = digitalRead(ENC_A[i]) ? 1 : 0;
    uint8_t b = digitalRead(ENC_B[i]) ? 1 : 0;
    encoderState[i] = (a << 1) | b;
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrEnc0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), isrEnc0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrEnc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), isrEnc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrEnc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[2]), isrEnc2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrEnc3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[3]), isrEnc3B, CHANGE);
}

void setup() {
  pinMode(SHIELD_ENABLE_PIN, OUTPUT);
  digitalWrite(SHIELD_ENABLE_PIN, LOW);
  stopAllMotors();

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  CommandSerial.setRx(PA10);
  CommandSerial.setTx(PA9);
  CommandSerial.begin(BAUD_RATE);
  delay(100);

  setupEncoders();
  digitalWrite(SHIELD_ENABLE_PIN, HIGH);
  lastMotionCommandMs = millis();
  lastControlMs = millis();
  lastTelemMs = millis();

  CommandSerial.println("READY four_wheel_encoder_demo motors_disabled");
  printHelp();
}

void updateHeartbeat() {
  uint32_t now = millis();
  if (now - lastHeartbeatMs < 500) return;
  lastHeartbeatMs = now;
  heartbeatLedOn = !heartbeatLedOn;
  digitalWrite(STATUS_LED_PIN, heartbeatLedOn ? LOW : HIGH);
}

void loop() {
  readSerialLines();
  updateControl();
  printTelemetry();
  updateHeartbeat();
}
