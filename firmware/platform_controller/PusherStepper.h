#ifndef PUSHER_STEPPER_H
#define PUSHER_STEPPER_H

#include <Arduino.h>

class PusherStepper {
public:
  PusherStepper();

  void begin();
  void home();
  void pushOnce();
  void moveToMm(float targetMm);
  void moveRelativeMm(float deltaMm);
  float positionMm() const;

private:
  static const int STEP_PIN = 5;
  static const int DIR_PIN = 6;

  // TB6600 DIP is set to 800 pulse/rev, but the verified mm test moves
  // correctly with this calibrated value.
  static const long PHYSICAL_MICROSTEP_SETTING = 800L;
  static const long PULSES_PER_REV = 400L;
  static const long LEAD_MM_PER_REV = 4L;
  static const long PULSES_PER_MM = PULSES_PER_REV / LEAD_MM_PER_REV;
  static const long PUSH_DISTANCE_MM = 260L;
  static const long PUSH_STEPS = PUSH_DISTANCE_MM * PULSES_PER_MM;
  static const unsigned long RETURN_DELAY_MS = 1000;

  static const bool DIR_FORWARD = LOW;
  static const bool DIR_BACKWARD = HIGH;
  static const unsigned int PULSE_WIDTH_US = 10;
  static const unsigned int DIR_SETUP_US = 1000;
  static const unsigned long START_INTERVAL_US = 1000;
  static const unsigned long TARGET_INTERVAL_US = 200;
  static const long ACCEL_DISTANCE_MM = 30L;
  static const long ACCEL_PULSES = ACCEL_DISTANCE_MM * PULSES_PER_MM;
  static const float MIN_POSITION_MM;
  static const float MAX_POSITION_MM;

  float currentPositionMm = 0.0f;

  void movePulses(long pulses, bool direction);
  void moveDistanceMm(float distanceMm, bool direction);
  void outputOnePulse();
  unsigned long rampedInterval(long totalPulses, long remainingPulses, long completedPulses) const;
};

#endif
