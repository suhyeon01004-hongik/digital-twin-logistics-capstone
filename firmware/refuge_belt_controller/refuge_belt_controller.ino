/*
  Refuge circulation low-level controller.

  Role:
  - Drive B1..B4 motors with encoder-based RPM PI control.
  - Execute low-level MOVE belt dir mm commands using encoder distance.
  - Read VL53L0X ToF channels through TCA9548A.
  - Publish compact JSON telemetry over USB serial.

  High-level package DB, routing, target/refuge logic, and web UI control live
  on ROS 2 nodes. This sketch intentionally does not keep a box database.
*/

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define NUM_BELTS 4
#define NUM_TOF 8

static const long SERIAL_BAUD = 115200;
static const uint8_t FLOOR_ID = 1;
static const bool USE_ESTOP = true;
static const uint8_t ESTOP_PIN = 40;

static const uint8_t TCA9548A_ADDR = 0x70;
static const uint8_t VL53L0X_ADDR = 0x29;
static const uint32_t I2C_CLOCK_HZ = 100000UL;
static const unsigned long I2C_TIMEOUT_US = 200000UL;
static const uint16_t TOF_INVALID_MM = 8190;
static const int TOF_RAW_MIN_VALID_MM = 20;
static const int TOF_RAW_MAX_VALID_MM = 650;
static const unsigned long TOF_INVALID_HOLD_MS = 5000;
static const unsigned long TOF_REINIT_AFTER_INVALID_MS = 1500;

static const unsigned long SENSOR_PERIOD_MS = 50;
static const unsigned long TELEMETRY_PERIOD_MS = 200;
static const unsigned long PID_SAMPLE_TIME_MS = 100;
static const unsigned long JAM_TIMEOUT_MS = 1200;
static const unsigned long MAX_MOVE_TIME_MS = 180000;

// Encoder model:
// - Motor encoder: 5 pulses/rev on the motor shaft.
// - This firmware counts encoder A on CHANGE, so 5 pulses become 10 counts/rev.
// - Gear ratio: 54:1.
// - GT2 20T pulley: 20 teeth * 2 mm = 40 mm/rev.
// Therefore theoretical output pulley counts/rev = 5 * 2 * 54 = 540,
// and theoretical belt travel = 40 / 540 = 0.074074 mm/count.
// Field-calibrated defaults per belt. Both directions start from the same
// value and can still be tuned at runtime with SET MMCOUNT.
static const float ENCODER_PPR_FOR_RPM = 540.0f;
static float MM_PER_ENCODER_COUNT[NUM_BELTS][2] = {
  {0.125000f, 0.125500f},
  {0.126000f, 0.124522f},
  {0.126006f, 0.126006f},
  {0.126500f, 0.128100f}
};
static float MOVE_SCALE[NUM_BELTS][2] = {
  {1.0f, 1.0f},
  {1.0f, 1.0f},
  {1.0f, 1.0f},
  {1.0f, 1.0f}
};
static float MOVE_OFFSET_MM[NUM_BELTS][2] = {
  {0.0f, 0.0f},
  {0.0f, 0.0f},
  {0.0f, 0.0f},
  {0.0f, 0.0f}
};
static const int DIST_BIN_COUNT = 4;
static const float DIST_BIN_MAX_MM[DIST_BIN_COUNT] = {20.0f, 100.0f, 250.0f, 100000.0f};
static float DIST_MOVE_SCALE[NUM_BELTS][2][DIST_BIN_COUNT] = {
  {{0.45f, 0.93f, 0.94f, 0.92f}, {0.45f, 0.93f, 0.94f, 0.92f}},
  {{0.45f, 0.9762f, 0.94f, 0.952823f}, {0.45f, 0.9762f, 0.94f, 0.952823f}},
  {{0.45f, 0.93f, 0.94f, 0.92f}, {0.45f, 0.93f, 0.94f, 0.92f}},
  {{0.45f, 0.93f, 0.94f, 1.037917f}, {0.45f, 0.93f, 0.94f, 1.037917f}}
};
static float DIST_MOVE_OFFSET_MM[NUM_BELTS][2][DIST_BIN_COUNT] = {0};
static float defaultTargetRpm = 45.0f;
static float globalKp = 0.8f;
static float globalKi = 0.30f;
static float globalKd = 0.00f;
static const float RPM_INTEGRAL_LIMIT = 60.0f;
static float slowdownDistanceMm = 25.0f;
static float minMoveRpm = 25.0f;
static const int MIN_PWM = 35;
static const int MAX_PWM = 255;
static int pwmStepLimit = 25;

struct MotorPins {
  const char* name;
  uint8_t pwm;
  uint8_t dir;
  uint8_t encA;
  uint8_t encB;
  bool invertDir;
};

static MotorPins motorPins[NUM_BELTS] = {
  {"B1", 10, 11, 2,  4,  true},
  {"B2", 6,  7,  19, 23, true},
  {"B3", 8,  9,  18, 22, false},
  {"B4", 12, 13, 3,  5,  false}
};

static const int8_t FORWARD_SIGN[NUM_BELTS] = {1, 1, 1, 1};
static const char* const TOF_NAME[NUM_TOF] = {
  "F1_B1_ToF_empty_space",
  "F1_B1_ToF_Transfer",
  "F1_B2_ToF_empty_space",
  "F1_B2_ToF_Transfer",
  "F1_B3_ToF_empty_space",
  "F1_B3_ToF_Transfer",
  "F1_B4_ToF_empty_space",
  "F1_B4_ToF_Transfer"
};

// Logical ToF index -> physical TCA9548A mux channel.
// Keep CH6 and CH7 explicitly separated; they are B4 gap/transfer.
static const uint8_t TOF_TCA_CHANNEL[NUM_TOF] = {0, 1, 2, 3, 4, 5, 6, 7};
static const bool TOF_ENABLED[NUM_TOF] = {
  true, false, true, false, true, false, true, false
};

static const int TOF_CAL_N = 9;
static const float TOF_RAW_TABLE[TOF_CAL_N] = {
  84.0f, 130.0f, 174.0f, 230.0f, 296.0f,
  340.0f, 392.0f, 425.0f, 490.0f
};
static const float TOF_ACTUAL_TABLE[TOF_CAL_N] = {
  50.0f, 100.0f, 150.0f, 200.0f, 250.0f,
  300.0f, 350.0f, 400.0f, 450.0f
};
static const float TOF_KALMAN_Q = 1.0f;
static const float TOF_KALMAN_R = 2.0f;
static float tofDeadbandMm = 1.0f;

VL53L0X tof[NUM_TOF];
bool tofOk[NUM_TOF];
uint16_t tofMm[NUM_TOF];
uint16_t tofRawMm[NUM_TOF];
float tofXEst[NUM_TOF];
float tofPEst[NUM_TOF];
bool tofFilterInitialized[NUM_TOF];
bool tofOutputInitialized[NUM_TOF];
uint16_t tofStableOutputMm[NUM_TOF];
unsigned long tofLastValidMs[NUM_TOF];
unsigned long tofInvalidSinceMs[NUM_TOF];

volatile long encoderCount[NUM_BELTS] = {0, 0, 0, 0};
long lastPidCount[NUM_BELTS] = {0, 0, 0, 0};
float currentRpm[NUM_BELTS] = {0, 0, 0, 0};
float filteredRpm[NUM_BELTS] = {0, 0, 0, 0};
float targetRpm[NUM_BELTS] = {0, 0, 0, 0};
float rpmIntegral[NUM_BELTS] = {0, 0, 0, 0};
float rpmPrevError[NUM_BELTS] = {0, 0, 0, 0};
int outputPwm[NUM_BELTS] = {0, 0, 0, 0};
int motorDir[NUM_BELTS] = {1, 1, 1, 1};

bool faulted = false;
char faultText[48] = "";
bool useTof = true;

bool moving = false;
int activeBelt = -1;
int activeDir = 1;
float activeRequestedMm = 0.0f;
float activeTargetMm = 0.0f;
float activeCruiseRpm = 0.0f;
long activeStartCount = 0;
long activeLastCount = 0;
unsigned long activeStartMs = 0;
unsigned long activeLastChangeMs = 0;
bool activeTofStopEnabled = false;
int activeTofStopChannel = -1;
char activeTofStopMode = 'N';
float activeTofStopThreshold = 0.0f;
bool auxRunActive[NUM_BELTS] = {false, false, false, false};
unsigned long auxRunUntilMs[NUM_BELTS] = {0, 0, 0, 0};

unsigned long lastPidMs = 0;
unsigned long lastSensorMs = 0;
unsigned long lastTelemetryMs = 0;

void handleEncoder(uint8_t b) {
  int a = digitalRead(motorPins[b].encA);
  int bb = digitalRead(motorPins[b].encB);
  int step = (a == bb) ? 1 : -1;
  encoderCount[b] += step * FORWARD_SIGN[b];
}

void encB1() { handleEncoder(0); }
void encB2() { handleEncoder(1); }
void encB3() { handleEncoder(2); }
void encB4() { handleEncoder(3); }

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(3);
}

void tcaDisableAll() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  delay(2);
}

void writeI2CReg8(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool i2cAddressPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void softResetSelectedTof() {
  Wire.clearWireTimeoutFlag();
  writeI2CReg8(VL53L0X_ADDR, 0xBF, 0x00);
  delay(10);
  writeI2CReg8(VL53L0X_ADDR, 0xBF, 0x01);
  delay(80);
  Wire.clearWireTimeoutFlag();
}

void scanTofMuxChannels() {
  for (uint8_t mux = 0; mux < 8; mux++) {
    tcaSelect(mux);
    bool present = i2cAddressPresent(VL53L0X_ADDR);
    Serial.print(F("{\"event\":\"tof_mux_scan\",\"mux\":"));
    Serial.print(mux);
    Serial.print(F(",\"present\":"));
    Serial.print(present ? 1 : 0);
    Serial.println(F("}"));
  }
  tcaDisableAll();
}

uint8_t tofMuxChannel(int logicalIndex) {
  if (logicalIndex < 0 || logicalIndex >= NUM_TOF) return 255;
  return TOF_TCA_CHANNEL[logicalIndex];
}

float calibrateRawToActual(float raw) {
  if (raw <= TOF_RAW_TABLE[0]) {
    float ratio = (raw - TOF_RAW_TABLE[0]) / (TOF_RAW_TABLE[1] - TOF_RAW_TABLE[0]);
    return TOF_ACTUAL_TABLE[0] + ratio * (TOF_ACTUAL_TABLE[1] - TOF_ACTUAL_TABLE[0]);
  }
  if (raw >= TOF_RAW_TABLE[TOF_CAL_N - 1]) {
    int i = TOF_CAL_N - 2;
    float ratio = (raw - TOF_RAW_TABLE[i]) / (TOF_RAW_TABLE[i + 1] - TOF_RAW_TABLE[i]);
    return TOF_ACTUAL_TABLE[i] + ratio * (TOF_ACTUAL_TABLE[i + 1] - TOF_ACTUAL_TABLE[i]);
  }
  for (int i = 0; i < TOF_CAL_N - 1; i++) {
    if (raw >= TOF_RAW_TABLE[i] && raw <= TOF_RAW_TABLE[i + 1]) {
      float ratio = (raw - TOF_RAW_TABLE[i]) / (TOF_RAW_TABLE[i + 1] - TOF_RAW_TABLE[i]);
      return TOF_ACTUAL_TABLE[i] + ratio * (TOF_ACTUAL_TABLE[i + 1] - TOF_ACTUAL_TABLE[i]);
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

uint16_t applyTofFilter(int idx, uint16_t raw) {
  float calibrated = calibrateRawToActual((float)raw);
  uint16_t candidate = (uint16_t)(tofKalmanFilter(idx, calibrated) + 0.5f);
  if (!tofOutputInitialized[idx] || tofStableOutputMm[idx] == TOF_INVALID_MM) {
    tofOutputInitialized[idx] = true;
    tofStableOutputMm[idx] = candidate;
    return candidate;
  }
  int diff = (int)candidate - (int)tofStableOutputMm[idx];
  if (diff < 0) diff = -diff;
  if ((float)diff > tofDeadbandMm) tofStableOutputMm[idx] = candidate;
  return tofStableOutputMm[idx];
}

bool initOneTofChannel(int i, bool verbose) {
  if (i < 0 || i >= NUM_TOF || !TOF_ENABLED[i]) {
    if (i >= 0 && i < NUM_TOF) {
      tofOk[i] = false;
      tofRawMm[i] = TOF_INVALID_MM;
      tofMm[i] = TOF_INVALID_MM;
      tofLastValidMs[i] = 0;
      tofInvalidSinceMs[i] = 0;
    }
    return false;
  }
  uint8_t muxChannel = tofMuxChannel(i);
  if (muxChannel > 7) {
    tofOk[i] = false;
    return false;
  }
  tcaSelect(muxChannel);
  delay(30);
  bool presentBeforeReset = i2cAddressPresent(VL53L0X_ADDR);
  softResetSelectedTof();
  bool presentAfterReset = i2cAddressPresent(VL53L0X_ADDR);
  tof[i].setBus(&Wire);
  tof[i].setTimeout(500);
  bool ok = presentAfterReset && tof[i].init(true);
  if (ok) {
    tof[i].setMeasurementTimingBudget(50000);
    tof[i].startContinuous(0);
  }
  tofOk[i] = ok;
  tofRawMm[i] = TOF_INVALID_MM;
  tofMm[i] = TOF_INVALID_MM;
  tofLastValidMs[i] = 0;
  tofInvalidSinceMs[i] = 0;
  tofXEst[i] = 0.0f;
  tofPEst[i] = 100.0f;
  tofFilterInitialized[i] = false;
  tofOutputInitialized[i] = false;
  tofStableOutputMm[i] = TOF_INVALID_MM;
  if (verbose) {
    Serial.print(ok ? F("{\"event\":\"tof_init\",\"driver\":\"pololu\",\"channel\":") : F("{\"event\":\"tof_fail\",\"driver\":\"pololu\",\"channel\":"));
    Serial.print(i);
    Serial.print(F(",\"mux\":"));
    Serial.print(muxChannel);
    if (!ok) {
      Serial.print(F(",\"present_before_reset\":"));
      Serial.print(presentBeforeReset ? 1 : 0);
      Serial.print(F(",\"present_after_reset\":"));
      Serial.print(presentAfterReset ? 1 : 0);
    }
    Serial.println(F("}"));
  }
  return ok;
}

void initTofSensors() {
  for (int i = 0; i < NUM_TOF; i++) {
    initOneTofChannel(i, true);
    delay(50);
  }
}

void readTofSensors() {
  if (!useTof) return;
  static unsigned long lastRetryMs[NUM_TOF] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < NUM_TOF; i++) {
    if (!TOF_ENABLED[i]) {
      tofOk[i] = false;
      tofRawMm[i] = TOF_INVALID_MM;
      tofMm[i] = TOF_INVALID_MM;
      continue;
    }
    if (!tofOk[i]) {
      unsigned long now = millis();
      if (now - lastRetryMs[i] >= 1500) {
        lastRetryMs[i] = now;
        initOneTofChannel(i, false);
      }
      tofMm[i] = TOF_INVALID_MM;
      continue;
    }
    uint8_t muxChannel = tofMuxChannel(i);
    if (muxChannel > 7) {
      tofMm[i] = TOF_INVALID_MM;
      continue;
    }
    tcaSelect(muxChannel);
    delay(10);
    uint16_t raw = tof[i].readRangeContinuousMillimeters();
    bool timedOut = tof[i].timeoutOccurred();
    unsigned long now = millis();
    if (!timedOut &&
               raw >= TOF_RAW_MIN_VALID_MM &&
               raw < TOF_RAW_MAX_VALID_MM) {
      tofRawMm[i] = raw;
      tofMm[i] = applyTofFilter(i, raw);
      tofLastValidMs[i] = now;
      tofInvalidSinceMs[i] = 0;
    } else {
      tofRawMm[i] = TOF_INVALID_MM;
      if (tofInvalidSinceMs[i] == 0) tofInvalidSinceMs[i] = now;
      if (tofLastValidMs[i] > 0 &&
          now - tofLastValidMs[i] <= TOF_INVALID_HOLD_MS &&
          tofStableOutputMm[i] != TOF_INVALID_MM) {
        tofMm[i] = tofStableOutputMm[i];
      } else {
        tofMm[i] = TOF_INVALID_MM;
      }
      if (now - tofInvalidSinceMs[i] >= TOF_REINIT_AFTER_INVALID_MS) {
        initOneTofChannel(i, false);
        tofInvalidSinceMs[i] = now;
      }
    }
  }
}

void configureMotors() {
  for (int b = 0; b < NUM_BELTS; b++) {
    pinMode(motorPins[b].pwm, OUTPUT);
    pinMode(motorPins[b].dir, OUTPUT);
    pinMode(motorPins[b].encA, INPUT_PULLUP);
    pinMode(motorPins[b].encB, INPUT_PULLUP);
    analogWrite(motorPins[b].pwm, 0);
    digitalWrite(motorPins[b].dir, LOW);
  }
  lastPidMs = millis();
}

void attachEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(motorPins[0].encA), encB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorPins[1].encA), encB2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorPins[2].encA), encB3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorPins[3].encA), encB4, CHANGE);
}

void stopAllMotors();

void setFault(const char* text) {
  stopAllMotors();
  if (!faulted) {
    strncpy(faultText, text, sizeof(faultText) - 1);
    faultText[sizeof(faultText) - 1] = '\0';
    faulted = true;
    Serial.print(F("{\"event\":\"fault\",\"text\":\""));
    Serial.print(faultText);
    Serial.println(F("\"}"));
  }
}

void clearFault() {
  faulted = false;
  faultText[0] = '\0';
}

void startMotorByRpm(int belt, int dir, float rpm) {
  if (belt < 0 || belt >= NUM_BELTS) return;
  bool dirState = (dir > 0) ? LOW : HIGH;
  if (motorPins[belt].invertDir) dirState = !dirState;
  digitalWrite(motorPins[belt].dir, dirState);
  motorDir[belt] = (dir > 0) ? 1 : -1;
  targetRpm[belt] = fabs(rpm);
  noInterrupts();
  lastPidCount[belt] = encoderCount[belt];
  interrupts();
  currentRpm[belt] = 0.0f;
  filteredRpm[belt] = 0.0f;
  rpmIntegral[belt] = 0.0f;
  rpmPrevError[belt] = 0.0f;
  outputPwm[belt] = MIN_PWM;
  analogWrite(motorPins[belt].pwm, outputPwm[belt]);
}

void stopMotorOutput(int belt) {
  if (belt < 0 || belt >= NUM_BELTS) return;
  targetRpm[belt] = 0.0f;
  outputPwm[belt] = 0;
  analogWrite(motorPins[belt].pwm, 0);
}

void clearAuxRunState(int belt) {
  if (belt < 0 || belt >= NUM_BELTS) return;
  auxRunActive[belt] = false;
  auxRunUntilMs[belt] = 0;
}

void clearAllAuxRunStates() {
  for (int b = 0; b < NUM_BELTS; b++) clearAuxRunState(b);
}

void stopNonAuxMotorsExcept(int keepBelt) {
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b == keepBelt) {
      clearAuxRunState(b);
      stopMotorOutput(b);
      continue;
    }
    if (!auxRunActive[b]) stopMotorOutput(b);
  }
}

void stopAllMotors() {
  for (int b = 0; b < NUM_BELTS; b++) stopMotorOutput(b);
  clearAllAuxRunStates();
  moving = false;
  activeBelt = -1;
  activeTofStopEnabled = false;
  activeTofStopChannel = -1;
  activeTofStopMode = 'N';
}

bool estopIsActive() {
  return USE_ESTOP && digitalRead(ESTOP_PIN) == LOW;
}

bool motionAllowed() {
  if (estopIsActive()) {
    stopAllMotors();
    setFault("ESTOP");
    Serial.println(F("{\"event\":\"motion_rejected\",\"reason\":\"estop\"}"));
    return false;
  }
  if (faulted) {
    Serial.println(F("{\"event\":\"motion_rejected\",\"reason\":\"fault_latched\"}"));
    return false;
  }
  return true;
}

void updateMotorPi() {
  unsigned long now = millis();
  if (now - lastPidMs < PID_SAMPLE_TIME_MS) return;
  float dtMin = (now - lastPidMs) / 60000.0f;
  float dtSec = (now - lastPidMs) / 1000.0f;
  lastPidMs = now;
  if (dtMin <= 0.0f || dtSec <= 0.0f) return;

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
    rpmIntegral[b] = constrain(rpmIntegral[b], -RPM_INTEGRAL_LIMIT, RPM_INTEGRAL_LIMIT);
    float derivative = (error - rpmPrevError[b]) / dtSec;
    rpmPrevError[b] = error;
    float control = globalKp * error + globalKi * rpmIntegral[b] + globalKd * derivative;
    int desiredPwm = outputPwm[b] + (int)control;
    if (desiredPwm > outputPwm[b] + pwmStepLimit) desiredPwm = outputPwm[b] + pwmStepLimit;
    if (desiredPwm < outputPwm[b] - pwmStepLimit) desiredPwm = outputPwm[b] - pwmStepLimit;
    outputPwm[b] = constrain(desiredPwm, MIN_PWM, MAX_PWM);
    analogWrite(motorPins[b].pwm, outputPwm[b]);
  }
}

long encoderSnapshot(int belt) {
  long countNow;
  noInterrupts();
  countNow = encoderCount[belt];
  interrupts();
  return countNow;
}

int dirToIndex(int dir) {
  return dir >= 0 ? 0 : 1;
}

float mmPerCountFor(int belt, int dir) {
  return MM_PER_ENCODER_COUNT[belt][dirToIndex(dir)];
}

int distanceBinIndex(float requestedMm) {
  for (int i = 0; i < DIST_BIN_COUNT; i++) {
    if (requestedMm <= DIST_BIN_MAX_MM[i]) return i;
  }
  return DIST_BIN_COUNT - 1;
}

float compensatedTargetMm(int belt, int dir, float requestedMm) {
  int di = dirToIndex(dir);
  int bin = distanceBinIndex(requestedMm);
  float target = requestedMm * MOVE_SCALE[belt][di] * DIST_MOVE_SCALE[belt][di][bin]
    + MOVE_OFFSET_MM[belt][di]
    + DIST_MOVE_OFFSET_MM[belt][di][bin];
  return max(0.0f, target);
}

float rpmForMoveDistance(float requestedRpm, float targetMm) {
  float rpm = requestedRpm > 0.0f ? requestedRpm : defaultTargetRpm;
  return rpm;
}

bool tofStopConditionMet() {
  if (!activeTofStopEnabled) return false;
  if (!useTof) return false;
  if (activeTofStopChannel < 0 || activeTofStopChannel >= NUM_TOF) return false;
  if (!tofOk[activeTofStopChannel]) return false;
  uint16_t value = tofMm[activeTofStopChannel];
  if (value == TOF_INVALID_MM) return false;
  if (activeTofStopMode == 'B') return (float)value <= activeTofStopThreshold;
  if (activeTofStopMode == 'E') return (float)value >= activeTofStopThreshold;
  return false;
}

void printMoveDone(long countDelta, float traveled, const char* stopReason) {
  Serial.print(F("{\"event\":\"move_done\",\"belt\":"));
  Serial.print(activeBelt + 1);
  Serial.print(F(",\"dir\":"));
  Serial.print(activeDir);
  Serial.print(F(",\"requested_mm\":"));
  Serial.print(activeRequestedMm, 1);
  Serial.print(F(",\"target_mm\":"));
  Serial.print(activeTargetMm, 1);
  Serial.print(F(",\"counts\":"));
  Serial.print(countDelta);
  Serial.print(F(",\"traveled_mm\":"));
  Serial.print(traveled, 1);
  if (stopReason && stopReason[0]) {
    Serial.print(F(",\"stop_reason\":\""));
    Serial.print(stopReason);
    Serial.print(F("\""));
  }
  if (activeTofStopEnabled) {
    Serial.print(F(",\"tof_channel\":"));
    Serial.print(activeTofStopChannel);
    Serial.print(F(",\"tof_mode\":\""));
    Serial.print(activeTofStopMode == 'B' ? "box" : "empty");
    Serial.print(F("\",\"tof_threshold\":"));
    Serial.print(activeTofStopThreshold, 1);
    Serial.print(F(",\"tof_value\":"));
    if (activeTofStopChannel >= 0 && activeTofStopChannel < NUM_TOF) Serial.print(tofMm[activeTofStopChannel]);
    else Serial.print(TOF_INVALID_MM);
  }
  Serial.println(F("}"));
}

void clearActiveMoveState() {
  moving = false;
  activeBelt = -1;
  activeTofStopEnabled = false;
  activeTofStopChannel = -1;
  activeTofStopMode = 'N';
}

void startMove(int belt, int dir, float mm, float rpm, bool tofStopEnabled = false,
               int tofStopChannel = -1, char tofStopMode = 'N', float tofStopThreshold = 0.0f) {
  if (!motionAllowed()) return;
  int moveDir = (dir > 0) ? 1 : -1;
  if (belt < 0 || belt >= NUM_BELTS || mm <= 0.0f) {
    Serial.print(F("{\"event\":\"bad_move_detail\",\"reason\":\"bad_args\",\"belt\":"));
    Serial.print(belt + 1);
    Serial.print(F(",\"dir\":"));
    Serial.print(dir);
    Serial.print(F(",\"mm\":"));
    Serial.print(mm, 3);
    Serial.println(F("}"));
    setFault("BAD_MOVE");
    return;
  }
  float targetMm = compensatedTargetMm(belt, moveDir, mm);
  if (targetMm <= 0.0f || mmPerCountFor(belt, moveDir) <= 0.0f) {
    Serial.print(F("{\"event\":\"bad_move_detail\",\"reason\":\"bad_calibration\",\"belt\":"));
    Serial.print(belt + 1);
    Serial.print(F(",\"dir\":"));
    Serial.print(moveDir);
    Serial.print(F(",\"requested_mm\":"));
    Serial.print(mm, 3);
    Serial.print(F(",\"target_mm\":"));
    Serial.print(targetMm, 3);
    Serial.print(F(",\"mm_per_count\":"));
    Serial.print(mmPerCountFor(belt, moveDir), 6);
    Serial.print(F(",\"move_scale\":"));
    Serial.print(MOVE_SCALE[belt][dirToIndex(moveDir)], 6);
    Serial.print(F(",\"dist_scale\":"));
    Serial.print(DIST_MOVE_SCALE[belt][dirToIndex(moveDir)][distanceBinIndex(mm)], 6);
    Serial.println(F("}"));
    setFault("BAD_MOVE");
    return;
  }
  stopNonAuxMotorsExcept(belt);
  moving = true;
  activeBelt = belt;
  activeDir = moveDir;
  activeRequestedMm = mm;
  activeTargetMm = targetMm;
  activeCruiseRpm = rpmForMoveDistance(rpm, targetMm);
  activeStartCount = encoderSnapshot(belt);
  activeLastCount = activeStartCount;
  activeStartMs = millis();
  activeLastChangeMs = activeStartMs;
  activeTofStopEnabled = tofStopEnabled && tofStopChannel >= 0 && tofStopChannel < NUM_TOF
    && (tofStopMode == 'B' || tofStopMode == 'E');
  activeTofStopChannel = activeTofStopEnabled ? tofStopChannel : -1;
  activeTofStopMode = activeTofStopEnabled ? tofStopMode : 'N';
  activeTofStopThreshold = tofStopThreshold;
  startMotorByRpm(belt, activeDir, activeCruiseRpm);
  Serial.print(F("{\"event\":\"move_start\",\"belt\":"));
  Serial.print(belt + 1);
  Serial.print(F(",\"dir\":"));
  Serial.print(activeDir);
  Serial.print(F(",\"mm\":"));
  Serial.print(mm, 1);
  Serial.print(F(",\"target_mm\":"));
  Serial.print(activeTargetMm, 1);
  if (activeTofStopEnabled) {
    Serial.print(F(",\"tof_channel\":"));
    Serial.print(activeTofStopChannel);
    Serial.print(F(",\"tof_mode\":\""));
    Serial.print(activeTofStopMode == 'B' ? "box" : "empty");
    Serial.print(F("\",\"tof_threshold\":"));
    Serial.print(activeTofStopThreshold, 1);
  }
  Serial.println(F("}"));
}

void updateMove() {
  if (!moving || activeBelt < 0) return;
  unsigned long now = millis();
  long current = encoderSnapshot(activeBelt);
  if (current != activeLastCount) {
    activeLastCount = current;
    activeLastChangeMs = now;
  }

  long countDelta = labs(current - activeStartCount);
  float traveled = countDelta * mmPerCountFor(activeBelt, activeDir);
  float remaining = activeTargetMm - traveled;
  if (remaining > 0.0f && remaining < slowdownDistanceMm) {
    float ratio = remaining / slowdownDistanceMm;
    targetRpm[activeBelt] = max(minMoveRpm, activeCruiseRpm * ratio);
  }

  if (tofStopConditionMet()) {
    stopMotorOutput(activeBelt);
    printMoveDone(countDelta, traveled, "tof");
    clearActiveMoveState();
    return;
  }

  if (traveled >= activeTargetMm) {
    stopMotorOutput(activeBelt);
    printMoveDone(countDelta, traveled, "encoder");
    clearActiveMoveState();
    return;
  }

  if (now - activeLastChangeMs > JAM_TIMEOUT_MS && now - activeStartMs > 500) {
    stopAllMotors();
    setFault("ENCODER_JAM");
    return;
  }

  if (now - activeStartMs > MAX_MOVE_TIME_MS) {
    stopAllMotors();
    setFault("MOVE_TIMEOUT");
  }
}

void startAuxRun(int belt, int dir, float rpm, unsigned long durationMs) {
  if (!motionAllowed()) return;
  int moveDir = (dir > 0) ? 1 : -1;
  if (belt < 0 || belt >= NUM_BELTS || rpm <= 0.0f || durationMs == 0) {
    setFault("BAD_AUXRUN");
    return;
  }
  if (moving && activeBelt == belt) {
    setFault("AUXRUN_BUSY");
    return;
  }
  auxRunActive[belt] = true;
  auxRunUntilMs[belt] = millis() + durationMs;
  startMotorByRpm(belt, moveDir, rpm);
  Serial.print(F("{\"event\":\"aux_run_start\",\"belt\":"));
  Serial.print(belt + 1);
  Serial.print(F(",\"dir\":"));
  Serial.print(moveDir);
  Serial.print(F(",\"rpm\":"));
  Serial.print(rpm, 1);
  Serial.print(F(",\"duration_ms\":"));
  Serial.print(durationMs);
  Serial.println(F("}"));
}

void updateAuxRuns() {
  unsigned long now = millis();
  for (int b = 0; b < NUM_BELTS; b++) {
    if (!auxRunActive[b]) continue;
    if ((long)(now - auxRunUntilMs[b]) < 0) continue;
    auxRunActive[b] = false;
    auxRunUntilMs[b] = 0;
    if (!(moving && activeBelt == b)) stopMotorOutput(b);
    Serial.print(F("{\"event\":\"aux_run_done\",\"belt\":"));
    Serial.print(b + 1);
    Serial.println(F("}"));
  }
}

void zeroEncoders() {
  noInterrupts();
  for (int b = 0; b < NUM_BELTS; b++) {
    encoderCount[b] = 0;
    lastPidCount[b] = 0;
  }
  interrupts();
  Serial.println(F("{\"event\":\"zero\"}"));
}

void printEncoderDebug() {
  Serial.print(F("{\"event\":\"enc_debug\",\"enc\":["));
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b) Serial.print(',');
    Serial.print(encoderSnapshot(b));
  }
  Serial.print(F("],\"pins\":["));
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b) Serial.print(',');
    Serial.print(F("{\"belt\":"));
    Serial.print(b + 1);
    Serial.print(F(",\"a_pin\":"));
    Serial.print(motorPins[b].encA);
    Serial.print(F(",\"a\":"));
    Serial.print(digitalRead(motorPins[b].encA));
    Serial.print(F(",\"b_pin\":"));
    Serial.print(motorPins[b].encB);
    Serial.print(F(",\"b\":"));
    Serial.print(digitalRead(motorPins[b].encB));
    Serial.print(F("}"));
  }
  Serial.println(F("]}"));
}

void printTelemetry() {
  Serial.print(F("{\"type\":\"telemetry\",\"floor\":"));
  Serial.print(FLOOR_ID);
  Serial.print(F(",\"ms\":"));
  Serial.print(millis());
  Serial.print(F(",\"fault\":"));
  Serial.print(faulted ? 1 : 0);
  Serial.print(F(",\"fault_text\":\""));
  Serial.print(faultText);
  Serial.print(F("\",\"estop\":"));
  Serial.print((USE_ESTOP && digitalRead(ESTOP_PIN) == LOW) ? 1 : 0);
  Serial.print(F(",\"moving\":"));
  Serial.print(moving ? 1 : 0);
  Serial.print(F(",\"active_belt\":"));
  Serial.print(activeBelt + 1);
  Serial.print(F(",\"active_dir\":"));
  Serial.print(activeDir);

  Serial.print(F(",\"enc\":["));
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b) Serial.print(',');
    Serial.print(encoderSnapshot(b));
  }
  Serial.print(F("],\"rpm\":["));
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b) Serial.print(',');
    Serial.print(currentRpm[b], 2);
  }
  Serial.print(F("],\"pwm\":["));
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b) Serial.print(',');
    Serial.print(outputPwm[b]);
  }
  Serial.print(F("],\"dir\":["));
  for (int b = 0; b < NUM_BELTS; b++) {
    if (b) Serial.print(',');
    Serial.print(motorDir[b]);
  }
  Serial.print(F("],\"tof_deadband_mm\":"));
  Serial.print(tofDeadbandMm, 2);
  Serial.print(F(",\"slowdown_mm\":"));
  Serial.print(slowdownDistanceMm, 2);
  Serial.print(F(",\"min_move_rpm\":"));
  Serial.print(minMoveRpm, 2);
  Serial.print(F(",\"pwm_step\":"));
  Serial.print(pwmStepLimit);
  Serial.print(F(",\"tof\":["));
  for (int i = 0; i < NUM_TOF; i++) {
    if (i) Serial.print(',');
    Serial.print(tofMm[i]);
  }
  Serial.print(F("],\"tof_ok\":["));
  for (int i = 0; i < NUM_TOF; i++) {
    if (i) Serial.print(',');
    Serial.print(tofOk[i] ? 1 : 0);
  }
  Serial.println(F("]}"));
}

int beltNoToIndex(int beltNo) {
  if (beltNo < 1 || beltNo > NUM_BELTS) return -1;
  return beltNo - 1;
}

void toUpperInPlace(char* s) {
  for (; *s; s++) {
    if (*s >= 'a' && *s <= 'z') *s = *s - 'a' + 'A';
  }
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  toUpperInPlace(cmd);

  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("{\"event\":\"pong\"}"));
  } else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "STOP_ALL") == 0) {
    stopAllMotors();
    Serial.println(F("{\"event\":\"stop_all\"}"));
  } else if (strcmp(cmd, "STOPB") == 0) {
    char* beltTok = strtok(NULL, " ");
    int belt = beltNoToIndex(beltTok ? atoi(beltTok) : 0);
    stopMotorOutput(belt);
    clearAuxRunState(belt);
    if (belt == activeBelt) {
      moving = false;
      activeBelt = -1;
    }
    Serial.println(F("{\"event\":\"stop_belt\"}"));
  } else if (strcmp(cmd, "ZERO") == 0) {
    zeroEncoders();
  } else if (strcmp(cmd, "TEL") == 0 || strcmp(cmd, "STATUS") == 0) {
    printTelemetry();
  } else if (strcmp(cmd, "ENCDBG") == 0) {
    printEncoderDebug();
  } else if (strcmp(cmd, "CLEAR_FAULT") == 0) {
    if (estopIsActive()) {
      stopAllMotors();
      setFault("ESTOP");
      Serial.println(F("{\"event\":\"clear_fault_rejected\",\"reason\":\"estop\"}"));
    } else {
      clearFault();
      Serial.println(F("{\"event\":\"clear_fault\"}"));
    }
  } else if (strcmp(cmd, "RUN") == 0) {
    char* beltTok = strtok(NULL, " ");
    char* dirTok = strtok(NULL, " ");
    char* rpmTok = strtok(NULL, " ");
    int belt = beltNoToIndex(beltTok ? atoi(beltTok) : 0);
    int dir = dirTok ? atoi(dirTok) : 1;
    float rpm = rpmTok ? atof(rpmTok) : defaultTargetRpm;
    if (belt < 0 || (dir != 1 && dir != -1) || rpm <= 0.0f) {
      setFault("BAD_RUN");
      return;
    }
    if (!motionAllowed()) return;
    moving = false;
    activeBelt = -1;
    clearAuxRunState(belt);
    startMotorByRpm(belt, dir, rpm);
    Serial.println(F("{\"event\":\"run\"}"));
  } else if (strcmp(cmd, "AUXRUN") == 0) {
    char* beltTok = strtok(NULL, " ");
    char* dirTok = strtok(NULL, " ");
    char* rpmTok = strtok(NULL, " ");
    char* durationTok = strtok(NULL, " ");
    int belt = beltNoToIndex(beltTok ? atoi(beltTok) : 0);
    int dir = dirTok ? atoi(dirTok) : 1;
    float rpm = rpmTok ? atof(rpmTok) : defaultTargetRpm;
    unsigned long durationMs = durationTok ? strtoul(durationTok, NULL, 10) : 0;
    startAuxRun(belt, dir, rpm, durationMs);
  } else if (strcmp(cmd, "MOVE") == 0) {
    char* beltTok = strtok(NULL, " ");
    char* dirTok = strtok(NULL, " ");
    char* mmTok = strtok(NULL, " ");
    char* rpmTok = strtok(NULL, " ");
    int belt = beltNoToIndex(beltTok ? atoi(beltTok) : 0);
    int dir = dirTok ? atoi(dirTok) : 1;
    float mm = mmTok ? atof(mmTok) : 0.0f;
    float rpm = rpmTok ? atof(rpmTok) : defaultTargetRpm;
    bool tofStopEnabled = false;
    int tofStopChannel = -1;
    char tofStopMode = 'N';
    float tofStopThreshold = 0.0f;
    char* optTok = strtok(NULL, " ");
    if (optTok) {
      toUpperInPlace(optTok);
      if (strcmp(optTok, "TOF") == 0 || strcmp(optTok, "TOFSTOP") == 0) {
        char* chTok = strtok(NULL, " ");
        char* modeTok = strtok(NULL, " ");
        char* thresholdTok = strtok(NULL, " ");
        if (chTok && modeTok && thresholdTok) {
          tofStopChannel = atoi(chTok);
          if (modeTok[0] == 'b' || modeTok[0] == 'B') tofStopMode = 'B';
          else if (modeTok[0] == 'e' || modeTok[0] == 'E') tofStopMode = 'E';
          tofStopThreshold = atof(thresholdTok);
          tofStopEnabled = tofStopChannel >= 0 && tofStopChannel < NUM_TOF
            && (tofStopMode == 'B' || tofStopMode == 'E');
        }
      }
    }
    startMove(belt, dir, mm, rpm, tofStopEnabled, tofStopChannel, tofStopMode, tofStopThreshold);
  } else if (strcmp(cmd, "SET") == 0) {
    char* key = strtok(NULL, " ");
    char* val = strtok(NULL, " ");
    if (!key || !val) {
      Serial.println(F("{\"event\":\"set_ignored\",\"reason\":\"missing_key_or_value\"}"));
      return;
    }
    toUpperInPlace(key);
    if (strcmp(key, "RPM") == 0) defaultTargetRpm = constrain(atof(val), 1.0f, 200.0f);
    else if (strcmp(key, "KP") == 0) globalKp = atof(val);
    else if (strcmp(key, "KI") == 0) globalKi = atof(val);
    else if (strcmp(key, "KD") == 0) globalKd = atof(val);
    else if (strcmp(key, "SLOWDOWN") == 0) slowdownDistanceMm = constrain(atof(val), 20.0f, 300.0f);
    else if (strcmp(key, "MINRPM") == 0) minMoveRpm = constrain(atof(val), 0.5f, 80.0f);
    else if (strcmp(key, "PWMSTEP") == 0) pwmStepLimit = constrain(atoi(val), 1, 100);
    else if (strcmp(key, "TOF_DEADBAND") == 0 || strcmp(key, "TOFDEADBAND") == 0) tofDeadbandMm = constrain(atof(val), 0.0f, 50.0f);
    else if (strcmp(key, "TOF") == 0) useTof = atoi(val) != 0;
    else if (strcmp(key, "DISTBIN") == 0 || strcmp(key, "DISTANCE_BIN") == 0) {
      char* dirTok = strtok(NULL, " ");
      char* binTok = strtok(NULL, " ");
      char* scaleTok = strtok(NULL, " ");
      char* offsetTok = strtok(NULL, " ");
      int belt = beltNoToIndex(atoi(val));
      int dir = dirTok ? atoi(dirTok) : 1;
      int bin = binTok ? atoi(binTok) - 1 : -1;
      if (belt >= 0 && bin >= 0 && bin < DIST_BIN_COUNT && scaleTok) {
        int di = dirToIndex(dir);
        DIST_MOVE_SCALE[belt][di][bin] = constrain(atof(scaleTok), 0.1f, 2.0f);
        if (offsetTok) DIST_MOVE_OFFSET_MM[belt][di][bin] = constrain(atof(offsetTok), -100.0f, 100.0f);
      }
    }
    else if (strcmp(key, "MMCOUNT") == 0) {
      char* scaleTok = strtok(NULL, " ");
      char* dirTok = strtok(NULL, " ");
      int belt = beltNoToIndex(atoi(val));
      if (belt >= 0 && scaleTok) {
        float scale = constrain(atof(scaleTok), 0.001f, 10.0f);
        if (dirTok) MM_PER_ENCODER_COUNT[belt][dirToIndex(atoi(dirTok))] = scale;
        else {
          MM_PER_ENCODER_COUNT[belt][0] = scale;
          MM_PER_ENCODER_COUNT[belt][1] = scale;
        }
      }
    } else if (strcmp(key, "MOVE_SCALE") == 0 || strcmp(key, "MOVESCALE") == 0) {
      char* scaleTok = strtok(NULL, " ");
      char* dirTok = strtok(NULL, " ");
      int belt = beltNoToIndex(atoi(val));
      if (belt >= 0 && scaleTok) {
        float scale = constrain(atof(scaleTok), 0.1f, 2.0f);
        if (dirTok) MOVE_SCALE[belt][dirToIndex(atoi(dirTok))] = scale;
        else {
          MOVE_SCALE[belt][0] = scale;
          MOVE_SCALE[belt][1] = scale;
        }
      }
    } else if (strcmp(key, "MOVE_OFFSET") == 0 || strcmp(key, "MOVEOFFSET") == 0) {
      char* offsetTok = strtok(NULL, " ");
      char* dirTok = strtok(NULL, " ");
      int belt = beltNoToIndex(atoi(val));
      if (belt >= 0 && offsetTok) {
        float offset = atof(offsetTok);
        if (dirTok) MOVE_OFFSET_MM[belt][dirToIndex(atoi(dirTok))] = offset;
        else {
          MOVE_OFFSET_MM[belt][0] = offset;
          MOVE_OFFSET_MM[belt][1] = offset;
        }
      }
    } else if (strcmp(key, "DISTCAL") == 0) {
      char* dirTok = strtok(NULL, " ");
      char* scaleTok = strtok(NULL, " ");
      char* offsetTok = strtok(NULL, " ");
      int belt = beltNoToIndex(atoi(val));
      if (belt >= 0 && dirTok && scaleTok && offsetTok) {
        int di = dirToIndex(atoi(dirTok));
        MOVE_SCALE[belt][di] = constrain(atof(scaleTok), 0.1f, 2.0f);
        MOVE_OFFSET_MM[belt][di] = constrain(atof(offsetTok), -100.0f, 100.0f);
      }
    }
    Serial.println(F("{\"event\":\"set\"}"));
  } else {
    Serial.println(F("{\"event\":\"unknown_command\"}"));
  }
}

void readSerialCommand() {
  static char buf[128];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
      continue;
    }
    if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  if (USE_ESTOP) pinMode(ESTOP_PIN, INPUT_PULLUP);
  configureMotors();
  attachEncoderInterrupts();
  for (int i = 0; i < NUM_TOF; i++) {
    tofOk[i] = false;
    tofMm[i] = TOF_INVALID_MM;
    tofRawMm[i] = TOF_INVALID_MM;
    tofFilterInitialized[i] = false;
    tofOutputInitialized[i] = false;
    tofStableOutputMm[i] = TOF_INVALID_MM;
    tofLastValidMs[i] = 0;
    tofInvalidSinceMs[i] = 0;
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  tcaDisableAll();
  scanTofMuxChannels();
  if (useTof) initTofSensors();
  stopAllMotors();
  Serial.println(F("{\"event\":\"ready\",\"name\":\"refuge_low_level\"}"));
}

void loop() {
  readSerialCommand();
  if (estopIsActive()) {
    stopAllMotors();
    setFault("ESTOP");
  }

  unsigned long now = millis();
  updateMotorPi();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    readTofSensors();
  }

  updateMove();
  updateAuxRuns();

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    printTelemetry();
  }
}
