#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController {
public:
  static const int MIN_ANGLE = 0;
  static const int MAX_ANGLE = 180;
  static const int CENTER_ANGLE = 90;

  void begin();
  void moveTo(int targetAngle);
  void writePulse(int pulseUs);
  void moveToCenter();
  int currentAngle() const;
  int currentPulse() const;

private:
  static const int SERVO_PIN = 10;
  static const int MIN_PULSE = 500;
  static const int MAX_PULSE = 2500;
  static const int STEP_DELAY_MS = 25;
  static const int SETTLE_DELAY_MS = 80;

  Servo servo;
  int current = CENTER_ANGLE;
  int pulse = 1500;

  int angleToPulse(int angle) const;
};

#endif
