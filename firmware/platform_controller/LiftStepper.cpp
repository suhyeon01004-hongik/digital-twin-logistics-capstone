#include "LiftStepper.h"

LiftStepper::LiftStepper() {
}

void LiftStepper::begin() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_UP);
}

void LiftStepper::update() {
  if (!busy) {
    return;
  }

  unsigned long now = micros();
  if (now - lastStepTime < currentIntervalUs) {
    return;
  }
  lastStepTime = now;

  if (remainingPulses <= 0) {
    finishMove();
    return;
  }

  outputOnePulse();
  remainingPulses--;
  completedPulses++;
  currentIntervalUs = rampedInterval(remainingPulses, completedPulses);

  if (remainingPulses <= 0) {
    finishMove();
  }
}

bool LiftStepper::moveUp() {
  return startMove(DIR_UP, "Lift up");
}

bool LiftStepper::moveDown() {
  return startMove(DIR_DOWN, "Lift down");
}

bool LiftStepper::moveToFloor(int requestedFloor) {
  requestedFloor = constrain(requestedFloor, MIN_FLOOR, MAX_FLOOR);
  if (requestedFloor == currentFloor && !busy) {
    Serial.print("Lift floor ");
    Serial.print(currentFloor);
    Serial.println(" done");
    return true;
  }
  return startFloorMove(requestedFloor);
}

bool LiftStepper::jogMm(float deltaMm) {
  return startJogMove(deltaMm);
}

void LiftStepper::zeroOffset() {
  currentOffsetMm = 0.0f;
  Serial.println("Lift offset zeroed");
}

bool LiftStepper::isBusy() const {
  return busy;
}

int LiftStepper::floor() const {
  return currentFloor;
}

float LiftStepper::offsetMm() const {
  return currentOffsetMm;
}

bool LiftStepper::startMove(bool direction, const char *label) {
  if (busy) {
    Serial.println("Lift busy");
    return false;
  }

  int nextFloor = currentFloor + ((direction == DIR_UP) ? 1 : -1);
  if (nextFloor < MIN_FLOOR || nextFloor > MAX_FLOOR) {
    Serial.print("Lift limit reached: floor ");
    Serial.println(currentFloor);
    return false;
  }

  currentDir = direction;
  digitalWrite(DIR_PIN, currentDir);
  delayMicroseconds(DIR_SETUP_US);

  totalPulses = LIFT_STEPS;
  remainingPulses = totalPulses;
  completedPulses = 0;
  targetFloor = nextFloor;
  jogMove = false;
  pendingJogMm = 0.0f;
  currentIntervalUs = START_INTERVAL_US;
  lastStepTime = micros();
  busy = true;

  Serial.print(label);
  Serial.print(" started: ");
  Serial.print(LIFT_DISTANCE_MM);
  Serial.print(" mm, pulses=");
  Serial.println(LIFT_STEPS);
  return true;
}

bool LiftStepper::startFloorMove(int requestedFloor) {
  if (busy) {
    Serial.println("Lift busy");
    return false;
  }

  targetFloor = constrain(requestedFloor, MIN_FLOOR, MAX_FLOOR);
  long floorDelta = targetFloor - currentFloor;
  if (floorDelta == 0) {
    Serial.print("Lift floor ");
    Serial.print(currentFloor);
    Serial.println(" done");
    return true;
  }

  currentDir = (floorDelta > 0) ? DIR_UP : DIR_DOWN;
  digitalWrite(DIR_PIN, currentDir);
  delayMicroseconds(DIR_SETUP_US);

  totalPulses = abs(floorDelta) * LIFT_STEPS;
  remainingPulses = totalPulses;
  completedPulses = 0;
  jogMove = false;
  pendingJogMm = 0.0f;
  currentIntervalUs = START_INTERVAL_US;
  lastStepTime = micros();
  busy = true;

  Serial.print("Lift move started: floor ");
  Serial.print(currentFloor);
  Serial.print(" -> ");
  Serial.print(targetFloor);
  Serial.print(", pulses=");
  Serial.println(remainingPulses);
  return true;
}

bool LiftStepper::startJogMove(float deltaMm) {
  if (busy) {
    Serial.println("Lift busy");
    return false;
  }

  long pulses = lround(abs(deltaMm) * PULSES_PER_MM);
  if (pulses <= 0) {
    Serial.println("Lift jog done");
    return true;
  }

  currentDir = (deltaMm >= 0.0f) ? DIR_UP : DIR_DOWN;
  digitalWrite(DIR_PIN, currentDir);
  delayMicroseconds(DIR_SETUP_US);

  totalPulses = pulses;
  remainingPulses = totalPulses;
  completedPulses = 0;
  jogMove = true;
  pendingJogMm = deltaMm;
  targetFloor = currentFloor;
  currentIntervalUs = START_INTERVAL_US;
  lastStepTime = micros();
  busy = true;

  Serial.print("Lift jog started: ");
  Serial.print(deltaMm, 3);
  Serial.print(" mm, pulses=");
  Serial.println(remainingPulses);
  return true;
}

void LiftStepper::outputOnePulse() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(STEP_PIN, LOW);
}

void LiftStepper::finishMove() {
  busy = false;
  totalPulses = 0;
  remainingPulses = 0;
  completedPulses = 0;
  currentIntervalUs = START_INTERVAL_US;
  digitalWrite(STEP_PIN, LOW);

  if (jogMove) {
    currentOffsetMm += pendingJogMm;
    jogMove = false;
    pendingJogMm = 0.0f;
    Serial.print("Lift jog done: offset_mm=");
    Serial.println(currentOffsetMm, 3);
    return;
  }

  if (currentDir == DIR_UP) {
    currentFloor = targetFloor;
    Serial.println("Lift up done");
  } else {
    currentFloor = targetFloor;
    Serial.println("Lift down done");
  }
  Serial.print("Lift floor ");
  Serial.print(currentFloor);
  Serial.println(" done");
}

unsigned long LiftStepper::rampedInterval(
  long remaining,
  long completed
) const {
  long rampPulses = min(ACCEL_PULSES, max(1L, totalPulses / 2L));
  long rampProgress = min(max(0L, completed), max(0L, remaining));
  if (rampProgress >= rampPulses) {
    return TARGET_INTERVAL_US;
  }

  float ratio = (float)rampProgress / (float)rampPulses;
  float interval = (float)START_INTERVAL_US -
    ((float)(START_INTERVAL_US - TARGET_INTERVAL_US) * ratio);
  return (unsigned long)max((float)TARGET_INTERVAL_US, interval);
}
