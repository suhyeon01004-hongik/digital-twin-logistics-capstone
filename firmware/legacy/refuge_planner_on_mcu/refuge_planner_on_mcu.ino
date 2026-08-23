/*
  Modularized arduino_refuge_circulation_test_2.

  The refuge circulation planner and DB logic are kept from
  arduino_refuge_circulation_test_2.ino. Hardware pin map, motor control,
  and ToF filtering are adapted to the configured floor controller headers.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "Config.h"
#include "FunctionPrototypes.h"

// ============================================================
// Global state
// ============================================================
Box boxes[MAX_BOX];
uint8_t boxCount = 0;
uint16_t targetId = 0;
uint16_t completeTargetId = 0;
uint16_t refugeCount = 0;
uint16_t nextSeqId = 1;
uint16_t nextSeqOrder = 1;
uint8_t loadStageIndex = 0;

Adafruit_VL53L0X tof[NUM_TOF];
bool tofOk[NUM_TOF];
uint16_t tofMm[NUM_TOF];

uint16_t tofRawMm[NUM_TOF];
float tofCalibratedMm[NUM_TOF];
float tofKalmanMm[NUM_TOF];
float tofXEst[NUM_TOF];
float tofPEst[NUM_TOF];
bool tofFilterInitialized[NUM_TOF];
bool tofOutputInitialized[NUM_TOF];
uint16_t tofStableOutputMm[NUM_TOF];

volatile long encoderCount[NUM_BELTS] = {0, 0, 0, 0};

float currentRpm[NUM_BELTS] = {0, 0, 0, 0};
float filteredRpm[NUM_BELTS] = {0, 0, 0, 0};
float targetRpm[NUM_BELTS] = {0, 0, 0, 0};
float rpmIntegral[NUM_BELTS] = {0, 0, 0, 0};
float rpmPrevError[NUM_BELTS] = {0, 0, 0, 0};
int outputPwm[NUM_BELTS] = {0, 0, 0, 0};
long lastPidCount[NUM_BELTS] = {0, 0, 0, 0};
unsigned long lastPidMs = 0;

int activeBelt = -1;
int activeDir = 1;
bool autoMode = false;
bool faulted = false;
char faultText[64] = "";
char lastAutoReason[96] = "";
bool waitingManualRefuge = false;
uint16_t pendingRefugeId = 0;
unsigned long lastSensorMs = 0;
unsigned long lastStatusMs = 0;

static bool useTof = USE_TOF_DEFAULT;
static bool debugAuto = true;
static bool autoRefugeDrop = false;

// ============================================================
// Modules
// ============================================================
#include "SystemState.h"
#include "MotorControl.h"
#include "SensorUtils.h"
#include "BoxDatabase.h"
#include "MotionControl.h"
#include "RefugePlanner.h"
#include "CommandParser.h"

// ============================================================
// Arduino entry points
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  if (USE_ESTOP) {
    pinMode(ESTOP_PIN, INPUT_PULLUP);
  }

  configureMotors();
  attachEncoderInterrupts();

  for (int i = 0; i < NUM_TOF; i++) {
    tofOk[i] = false;
    tofMm[i] = TOF_INVALID_MM;
    tofRawMm[i] = TOF_INVALID_MM;
    tofCalibratedMm[i] = 0.0f;
    tofKalmanMm[i] = 0.0f;
    tofXEst[i] = 0.0f;
    tofPEst[i] = 100.0f;
    tofFilterInitialized[i] = false;
    tofOutputInitialized[i] = false;
    tofStableOutputMm[i] = TOF_INVALID_MM;
  }

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  if (useTof) {
    initTofSensors();
  }

  stopAllMotors();
  Serial.println(F("OK REFUGE_CIRCULATION_TEST READY. Type HELP."));
}

void loop() {
  readSerialCommand();

  if (USE_ESTOP && digitalRead(ESTOP_PIN) == LOW) {
    setFault("ESTOP");
  }

  unsigned long now = millis();

  updateMotorPi();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    readTofSensors();
  }

  if (autoMode && !faulted && activeBelt < 0) {
    autoStep();
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    printStatus();
  }
}
