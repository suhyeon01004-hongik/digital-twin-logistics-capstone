#include "ServoController.h"

void ServoController::begin() {
  servo.attach(SERVO_PIN, MIN_PULSE, MAX_PULSE);
  pulse = angleToPulse(current);
  servo.writeMicroseconds(pulse);
  delay(SETTLE_DELAY_MS);
}

void ServoController::moveTo(int targetAngle) {
  targetAngle = constrain(targetAngle, MIN_ANGLE, MAX_ANGLE);

  if (!servo.attached()) {
    servo.attach(SERVO_PIN, MIN_PULSE, MAX_PULSE);
  }

  if (targetAngle == current) {
    pulse = angleToPulse(current);
    servo.writeMicroseconds(pulse);
  } else {
    int step = (targetAngle > current) ? 1 : -1;

    while (current != targetAngle) {
      current += step;
      pulse = angleToPulse(current);
      servo.writeMicroseconds(pulse);
      delay(STEP_DELAY_MS);
    }
  }

  delay(SETTLE_DELAY_MS);

  current = targetAngle;
}

void ServoController::writePulse(int pulseUs) {
  pulse = constrain(pulseUs, MIN_PULSE, MAX_PULSE);
  if (!servo.attached()) {
    servo.attach(SERVO_PIN, MIN_PULSE, MAX_PULSE);
  }
  servo.writeMicroseconds(pulse);
  delay(SETTLE_DELAY_MS);
}

void ServoController::moveToCenter() {
  moveTo(CENTER_ANGLE);
}

int ServoController::currentAngle() const {
  return current;
}

int ServoController::currentPulse() const {
  return pulse;
}

int ServoController::angleToPulse(int angle) const {
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
  return map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE);
}
