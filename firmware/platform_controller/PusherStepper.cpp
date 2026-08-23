#include "PusherStepper.h"
#include <math.h>

const float PusherStepper::MIN_POSITION_MM = 0.0f;
const float PusherStepper::MAX_POSITION_MM = 420.0f;

PusherStepper::PusherStepper() {
}

void PusherStepper::begin() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_FORWARD);
}

void PusherStepper::home() {
  currentPositionMm = 0.0f;
  Serial.println("Pusher position zeroed");
}

void PusherStepper::pushOnce() {
  Serial.print("Push started: ");
  Serial.print(PUSH_DISTANCE_MM);
  Serial.print(" mm, pulses=");
  Serial.println(PUSH_STEPS);
  Serial.print("Physical microstep DIP: ");
  Serial.print(PHYSICAL_MICROSTEP_SETTING);
  Serial.print(", calibrated pulses/rev: ");
  Serial.println(PULSES_PER_REV);

  Serial.println("Forward started");
  moveDistanceMm(PUSH_DISTANCE_MM, DIR_FORWARD);
  Serial.println("Forward done");

  delay(RETURN_DELAY_MS);

  Serial.println("Return started");
  moveDistanceMm(PUSH_DISTANCE_MM, DIR_BACKWARD);

  Serial.println("Push done");
}

void PusherStepper::moveToMm(float targetMm) {
  targetMm = constrain(targetMm, MIN_POSITION_MM, MAX_POSITION_MM);
  float deltaMm = targetMm - currentPositionMm;
  if (fabs(deltaMm) < 0.05f) {
    Serial.print("Pusher move done: position_mm=");
    Serial.println(currentPositionMm, 2);
    return;
  }

  bool direction = deltaMm >= 0.0f ? DIR_FORWARD : DIR_BACKWARD;
  Serial.print("Pusher move started: target_mm=");
  Serial.print(targetMm, 2);
  Serial.print(", current_mm=");
  Serial.print(currentPositionMm, 2);
  Serial.print(", delta_mm=");
  Serial.println(deltaMm, 2);
  moveDistanceMm(fabs(deltaMm), direction);
  currentPositionMm = targetMm;
  Serial.print("Pusher move done: position_mm=");
  Serial.println(currentPositionMm, 2);
}

void PusherStepper::moveRelativeMm(float deltaMm) {
  moveToMm(currentPositionMm + deltaMm);
}

float PusherStepper::positionMm() const {
  return currentPositionMm;
}

void PusherStepper::moveDistanceMm(float distanceMm, bool direction) {
  long pulses = lround(max(0.0f, distanceMm) * PULSES_PER_MM);
  if (pulses <= 0) {
    return;
  }
  movePulses(pulses, direction);
  if (direction == DIR_FORWARD) {
    currentPositionMm = constrain(currentPositionMm + distanceMm, MIN_POSITION_MM, MAX_POSITION_MM);
  } else {
    currentPositionMm = constrain(currentPositionMm - distanceMm, MIN_POSITION_MM, MAX_POSITION_MM);
  }
}

void PusherStepper::movePulses(long pulses, bool direction) {
  digitalWrite(DIR_PIN, direction);
  delayMicroseconds(DIR_SETUP_US);

  unsigned long intervalUs = START_INTERVAL_US;
  for (long i = 0; i < pulses; ++i) {
    outputOnePulse();
    long completed = i + 1;
    long remaining = pulses - completed;
    intervalUs = rampedInterval(pulses, remaining, completed);
    delayMicroseconds(intervalUs);
  }
}

void PusherStepper::outputOnePulse() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(STEP_PIN, LOW);
}

unsigned long PusherStepper::rampedInterval(
  long totalPulses,
  long remainingPulses,
  long completedPulses
) const {
  long rampPulses = min(ACCEL_PULSES, max(1L, totalPulses / 2L));
  long rampProgress = min(max(0L, completedPulses), max(0L, remainingPulses));
  if (rampProgress >= rampPulses) {
    return TARGET_INTERVAL_US;
  }

  float ratio = (float)rampProgress / (float)rampPulses;
  float interval = (float)START_INTERVAL_US -
    ((float)(START_INTERVAL_US - TARGET_INTERVAL_US) * ratio);
  return (unsigned long)max((float)TARGET_INTERVAL_US, interval);
}
