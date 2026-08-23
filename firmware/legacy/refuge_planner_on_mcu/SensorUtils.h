#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

// ============================================================
// ToF sensor helpers and belt-index utilities.
// normal: raw -> calibration -> Kalman -> output deadband -> tofMm[]
// invalid / fail: TOF_INVALID_MM(8190)
// ============================================================

bool initOneTofChannel(int i, bool verbose) {
  if (i < 0 || i >= NUM_TOF) return false;

  tcaSelect(i);
  delay(30);

  bool ok = tof[i].begin();

  if (ok) {
    tofOk[i] = true;

    tofRawMm[i] = TOF_INVALID_MM;
    tofCalibratedMm[i] = 0.0f;
    tofKalmanMm[i] = 0.0f;
    tofXEst[i] = 0.0f;
    tofPEst[i] = 100.0f;
    tofFilterInitialized[i] = false;
    tofOutputInitialized[i] = false;
    tofStableOutputMm[i] = TOF_INVALID_MM;
    tofMm[i] = TOF_INVALID_MM;

    if (verbose) {
      Serial.print(F("OK TOF_INIT "));
      Serial.println(TOF_NAME[i]);
    }
  } else {
    tofOk[i] = false;
    tofRawMm[i] = TOF_INVALID_MM;
    tofMm[i] = TOF_INVALID_MM;

    if (verbose) {
      Serial.print(F("WARN TOF_FAIL "));
      Serial.println(TOF_NAME[i]);
    }
  }

  return ok;
}

void initTofSensors() {
  for (int i = 0; i < NUM_TOF; i++) {
    tofOk[i] = false;
    tofRawMm[i] = TOF_INVALID_MM;
    tofCalibratedMm[i] = 0.0f;
    tofKalmanMm[i] = 0.0f;
    tofXEst[i] = 0.0f;
    tofPEst[i] = 100.0f;
    tofFilterInitialized[i] = false;
    tofOutputInitialized[i] = false;
    tofStableOutputMm[i] = TOF_INVALID_MM;
    tofMm[i] = TOF_INVALID_MM;
  }

  for (int i = 0; i < NUM_TOF; i++) {
    initOneTofChannel(i, true);
    delay(50);
  }
}

void readTofSensors() {
  if (!useTof) return;

  static unsigned long lastRetryMs[NUM_TOF] = {0, 0, 0, 0, 0, 0, 0, 0};

  for (int i = 0; i < NUM_TOF; i++) {
    // 부팅 때 실패한 채널도 계속 재시도
    if (!tofOk[i]) {
      unsigned long now = millis();

      if (now - lastRetryMs[i] >= 1500) {
        lastRetryMs[i] = now;

        if (initOneTofChannel(i, false)) {
          Serial.print(F("OK TOF_RECOVER "));
          Serial.println(TOF_NAME[i]);
        }
      }

      tofMm[i] = TOF_INVALID_MM;
      continue;
    }

    tcaSelect(i);
    delay(5);

    VL53L0X_RangingMeasurementData_t measure;
    tof[i].rangingTest(&measure, false);

    if (measure.RangeStatus != 4 &&
        measure.RangeMilliMeter >= TOF_RAW_MIN_VALID_MM &&
        measure.RangeMilliMeter < TOF_RAW_MAX_VALID_MM) {
      tofRawMm[i] = measure.RangeMilliMeter;
      tofMm[i] = applyTofFilter(i, measure.RangeMilliMeter);
    } else {
      tofRawMm[i] = TOF_INVALID_MM;
      tofMm[i] = TOF_INVALID_MM;
    }
  }
}

float calibrateRawToActual(float raw) {
  if (raw <= TOF_RAW_TABLE[0]) {
    float x0 = TOF_RAW_TABLE[0];
    float x1 = TOF_RAW_TABLE[1];
    float y0 = TOF_ACTUAL_TABLE[0];
    float y1 = TOF_ACTUAL_TABLE[1];
    float ratio = (raw - x0) / (x1 - x0);
    return y0 + ratio * (y1 - y0);
  }

  if (raw >= TOF_RAW_TABLE[TOF_CAL_N - 1]) {
    float x0 = TOF_RAW_TABLE[TOF_CAL_N - 2];
    float x1 = TOF_RAW_TABLE[TOF_CAL_N - 1];
    float y0 = TOF_ACTUAL_TABLE[TOF_CAL_N - 2];
    float y1 = TOF_ACTUAL_TABLE[TOF_CAL_N - 1];
    float ratio = (raw - x0) / (x1 - x0);
    return y0 + ratio * (y1 - y0);
  }

  for (int i = 0; i < TOF_CAL_N - 1; i++) {
    if (raw >= TOF_RAW_TABLE[i] && raw <= TOF_RAW_TABLE[i + 1]) {
      float x0 = TOF_RAW_TABLE[i];
      float x1 = TOF_RAW_TABLE[i + 1];
      float y0 = TOF_ACTUAL_TABLE[i];
      float y1 = TOF_ACTUAL_TABLE[i + 1];
      float ratio = (raw - x0) / (x1 - x0);
      return y0 + ratio * (y1 - y0);
    }
  }

  return raw;
}

float tofKalmanFilter(int idx, float measurement) {
  if (!tofFilterInitialized[idx]) {
    tofXEst[idx] = measurement;
    tofPEst[idx] = 100.0f;
    tofFilterInitialized[idx] = true;
    return tofXEst[idx];
  }

  tofPEst[idx] += TOF_KALMAN_Q;

  float k = tofPEst[idx] / (tofPEst[idx] + TOF_KALMAN_R);
  tofXEst[idx] = tofXEst[idx] + k * (measurement - tofXEst[idx]);
  tofPEst[idx] = (1.0f - k) * tofPEst[idx];

  return tofXEst[idx];
}

uint16_t tofOutputDeadband(int idx, uint16_t candidateMm) {
  if (!tofOutputInitialized[idx] || tofStableOutputMm[idx] == TOF_INVALID_MM) {
    tofStableOutputMm[idx] = candidateMm;
    tofOutputInitialized[idx] = true;
    return tofStableOutputMm[idx];
  }

  int diff = (int)candidateMm - (int)tofStableOutputMm[idx];
  if (diff < 0) diff = -diff;

  if (diff <= (int)TOF_DEADBAND_MM) {
    return tofStableOutputMm[idx];
  }

  tofStableOutputMm[idx] = candidateMm;
  return tofStableOutputMm[idx];
}

uint16_t applyTofFilter(int idx, uint16_t raw) {
  float calibrated = calibrateRawToActual((float)raw);
  float kalman = tofKalmanFilter(idx, calibrated);
  uint16_t candidate = (uint16_t)(kalman + 0.5f);
  uint16_t output = tofOutputDeadband(idx, candidate);

  tofCalibratedMm[idx] = calibrated;
  tofKalmanMm[idx] = kalman;
  return output;
}

void tcaSelect(uint8_t channel) {
  if (channel >= NUM_TOF) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();

  delay(3);
}

int gapTofIndex(int belt) {
  return belt * 2;
}

int transferTofIndex(int belt) {
  return belt * 2 + 1;
}

int nextBelt(int belt) {
  if (belt < 0) return 0;
  return (belt + 1) % NUM_BELTS;
}

int prevBelt(int belt) {
  if (belt < 0) return NUM_BELTS - 1;
  return (belt + NUM_BELTS - 1) % NUM_BELTS;
}

int beltAfter(int belt, int n) {
  int b = belt;
  for (int i = 0; i < n; i++) b = nextBelt(b);
  return b;
}

int beltBefore(int belt, int n) {
  int b = belt;
  for (int i = 0; i < n; i++) b = prevBelt(b);
  return b;
}

int beltNoToIndex(int beltNo) {
  if (beltNo < 1 || beltNo > NUM_BELTS) return -1;
  return beltNo - 1;
}

#endif