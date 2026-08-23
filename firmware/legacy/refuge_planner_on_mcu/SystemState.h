#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

void stopAllMotors() {
  for (int b = 0; b < NUM_BELTS; b++) {
    targetRpm[b] = 0.0f;
    outputPwm[b] = 0;
    analogWrite(motorPins[b].pwm, 0);

    // IMPORTANT:
    // The current hardware does not use EN pins. en == NO_ENABLE_PIN(255).
    // Never call digitalWrite(255). On AVR this can access an invalid pin table
    // and cause unstable behaviour.
    if (USE_MOTOR_ENABLE && motorPins[b].en != NO_ENABLE_PIN) {
      digitalWrite(motorPins[b].en, LOW);
    }
  }
  activeBelt = -1;
}

void zeroEncoders() {
  noInterrupts();
  for (int b = 0; b < 4; b++) encoderCount[b] = 0;
  interrupts();
  Serial.println(F("OK ZERO"));
}

void setFault(const char* text) {
  if (!faulted) {
    strncpy(faultText, text, sizeof(faultText) - 1);
    faultText[sizeof(faultText) - 1] = '\0';
    Serial.print(F("FAULT "));
    Serial.println(faultText);
  }
  faulted = true;
  autoMode = false;
  stopAllMotors();
}

void clearFault() {
  faulted = false;
  faultText[0] = '\0';
}

#endif