#ifndef LIFT_STEPPER_H
#define LIFT_STEPPER_H

#include <Arduino.h>

class LiftStepper {
public:
  LiftStepper();

  void begin();
  void update();
  bool moveUp();
  bool moveDown();
  bool moveToFloor(int targetFloor);
  bool jogMm(float deltaMm);
  void zeroOffset();
  bool isBusy() const;
  int floor() const;
  float offsetMm() const;

private:
  static const int STEP_PIN = 3;
  static const int DIR_PIN = 4;

  static const long PULSES_PER_REV = 1600L;
  static const long LEAD_MM_PER_REV = 4L;
  static const long PULSES_PER_MM = PULSES_PER_REV / LEAD_MM_PER_REV;
  static const long LIFT_DISTANCE_MM = 270L;
  static const long LIFT_STEPS = LIFT_DISTANCE_MM * PULSES_PER_MM;
  static const int MIN_FLOOR = 1;
  static const int MAX_FLOOR = 3;

  static const bool DIR_UP = HIGH;    // clockwise, + direction
  static const bool DIR_DOWN = LOW;   // counter-clockwise, - direction
  static const unsigned int PULSE_WIDTH_US = 10;
  static const unsigned int DIR_SETUP_US = 1000;
  static const unsigned long START_INTERVAL_US = 1200;
  static const unsigned long TARGET_INTERVAL_US = 83;
  static const long ACCEL_DISTANCE_MM = 8L;
  static const long ACCEL_PULSES = ACCEL_DISTANCE_MM * PULSES_PER_MM;

  bool busy = false;
  bool currentDir = DIR_UP;
  int currentFloor = 1;
  int targetFloor = 1;
  long totalPulses = 0;
  long remainingPulses = 0;
  long completedPulses = 0;
  bool jogMove = false;
  float pendingJogMm = 0.0f;
  float currentOffsetMm = 0.0f;
  unsigned long currentIntervalUs = START_INTERVAL_US;
  unsigned long lastStepTime = 0;

  bool startMove(bool direction, const char *label);
  bool startFloorMove(int targetFloor);
  bool startJogMove(float deltaMm);
  void outputOnePulse();
  void finishMove();
  unsigned long rampedInterval(long remaining, long completed) const;
};

#endif
