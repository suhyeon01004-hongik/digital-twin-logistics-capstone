#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ============================================================
// Motor pin setup, encoder ISR, and RPM PI control.
// Planner logic is unchanged; only motor output control was adapted
// to the configured pin map and PI speed loop.
// ============================================================

void encB1() { handleEncoder(0); }
void encB2() { handleEncoder(1); }
void encB3() { handleEncoder(2); }
void encB4() { handleEncoder(3); }

void handleEncoder(uint8_t b) {
  int a = digitalRead(motorPins[b].encA);
  int bb = digitalRead(motorPins[b].encB);
  int step = (a == bb) ? 1 : -1;
  encoderCount[b] += step * FORWARD_SIGN[b];
}

void configureMotors() {
  for (int b = 0; b < NUM_BELTS; b++) {
    pinMode(motorPins[b].pwm, OUTPUT);
    pinMode(motorPins[b].dir, OUTPUT);
    pinMode(motorPins[b].encA, INPUT_PULLUP);
    pinMode(motorPins[b].encB, INPUT_PULLUP);

    if (USE_MOTOR_ENABLE && motorPins[b].en != NO_ENABLE_PIN) {
      pinMode(motorPins[b].en, OUTPUT);
      digitalWrite(motorPins[b].en, LOW);
    }

    analogWrite(motorPins[b].pwm, 0);
    digitalWrite(motorPins[b].dir, LOW);

    targetRpm[b] = 0.0f;
    currentRpm[b] = 0.0f;
    filteredRpm[b] = 0.0f;
    rpmIntegral[b] = 0.0f;
    rpmPrevError[b] = 0.0f;
    outputPwm[b] = 0;
    lastPidCount[b] = 0;
  }
  lastPidMs = millis();
}

void attachEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(motorPins[0].encA), encB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorPins[1].encA), encB2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorPins[2].encA), encB3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorPins[3].encA), encB4, CHANGE);
}

void startMotorByRpm(int belt, int dir, float rpm) {
  if (belt < 0 || belt >= NUM_BELTS) return;

  bool dirState = (dir > 0) ? LOW : HIGH;
  if (motorPins[belt].invertDir) {
    dirState = !dirState;
  }
  digitalWrite(motorPins[belt].dir, dirState);

  if (USE_MOTOR_ENABLE && motorPins[belt].en != NO_ENABLE_PIN) {
    digitalWrite(motorPins[belt].en, HIGH);
  }

  targetRpm[belt] = fabs(rpm);
  rpmIntegral[belt] = 0.0f;
  rpmPrevError[belt] = 0.0f;

  if (outputPwm[belt] < MIN_PWM) outputPwm[belt] = MIN_PWM;
  analogWrite(motorPins[belt].pwm, outputPwm[belt]);
}

void setMotorRunning(int belt, int dir, int pwm) {
  if (belt < 0 || belt >= NUM_BELTS) return;

  // Keep the original call signature. PWM is accepted for compatibility,
  // but actual speed is controlled by the configured RPM PI loop.
  (void)pwm;
  startMotorByRpm(belt, dir, defaultTargetRpm);
}

void updateMotorPi() {
  unsigned long now = millis();
  if (now - lastPidMs < PID_SAMPLE_TIME_MS) return;

  float dtMin = (now - lastPidMs) / 60000.0f;
  float dtSec = (now - lastPidMs) / 1000.0f;
  if (dtMin <= 0.0f || dtSec <= 0.0f) {
    lastPidMs = now;
    return;
  }

  lastPidMs = now;

  for (int b = 0; b < NUM_BELTS; b++) {
    long countNow;
    noInterrupts();
    countNow = encoderCount[b];
    interrupts();

    long delta = countNow - lastPidCount[b];
    lastPidCount[b] = countNow;

    float rpm = fabs((float)delta) / ENCODER_PPR_FOR_RPM / dtMin;
    filteredRpm[b] = 0.75f * filteredRpm[b] + 0.25f * rpm;
    currentRpm[b] = filteredRpm[b];

    if (targetRpm[b] <= 0.0f) {
      outputPwm[b] = 0;
      analogWrite(motorPins[b].pwm, 0);
      continue;
    }

    float error = targetRpm[b] - currentRpm[b];
    rpmIntegral[b] += error * dtSec;
    float derivative = (error - rpmPrevError[b]) / dtSec;
    rpmPrevError[b] = error;

    float control = GLOBAL_KP * error + GLOBAL_KI * rpmIntegral[b] + GLOBAL_KD * derivative;
    int desiredPwm = outputPwm[b] + (int)control;

    if (desiredPwm > outputPwm[b] + PWM_STEP_LIMIT) desiredPwm = outputPwm[b] + PWM_STEP_LIMIT;
    if (desiredPwm < outputPwm[b] - PWM_STEP_LIMIT) desiredPwm = outputPwm[b] - PWM_STEP_LIMIT;

    desiredPwm = constrain(desiredPwm, MIN_PWM, MAX_PWM);
    outputPwm[b] = desiredPwm;
    analogWrite(motorPins[b].pwm, outputPwm[b]);
  }
}

void stopMotorOutput(int belt) {
  if (belt < 0 || belt >= NUM_BELTS) return;

  targetRpm[belt] = 0.0f;
  outputPwm[belt] = 0;
  analogWrite(motorPins[belt].pwm, 0);

  if (USE_MOTOR_ENABLE && motorPins[belt].en != NO_ENABLE_PIN) {
    digitalWrite(motorPins[belt].en, LOW);
  }
}

#endif
