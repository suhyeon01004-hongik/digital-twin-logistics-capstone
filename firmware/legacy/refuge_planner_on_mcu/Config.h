#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// Serial / floor
// ============================================================
static const long SERIAL_BAUD = 115200;
static const uint8_t FLOOR_ID = 1;

// ============================================================
// Hardware
// Pin map follows the already-wired Mega2560 floor controller.
// Index order in code: B1, B2, B3, B4.
// ============================================================
#define NUM_BELTS 4
#define NUM_TOF 8

static const bool USE_ESTOP = true;
static const uint8_t ESTOP_PIN = 40;

static const bool USE_TOF_DEFAULT = true;
static const uint8_t TCA9548A_ADDR = 0x70;
static const uint32_t I2C_CLOCK_HZ = 400000UL;

static const bool USE_MOTOR_ENABLE = false;
static const uint8_t NO_ENABLE_PIN = 255;

struct MotorPins {
  const char* name;
  uint8_t pwm;
  uint8_t dir;
  uint8_t en;
  uint8_t encA;
  uint8_t encB;
  bool invertDir;
};

static MotorPins motorPins[NUM_BELTS] = {
  {"B1", 10, 11, NO_ENABLE_PIN, 2,  4,  true},
  {"B2", 6,  7,  NO_ENABLE_PIN, 19, 23, true},
  {"B3", 8,  9,  NO_ENABLE_PIN, 18, 22, false},
  {"B4", 12, 13, NO_ENABLE_PIN, 3,  5,  false}
};

static const int8_t FORWARD_SIGN[NUM_BELTS] = {1, 1, 1, 1};

// ============================================================
// ToF channel map
// ============================================================
enum TofIndex {
  B1_GAP = 0, B1_TRANSFER = 1,
  B2_GAP = 2, B2_TRANSFER = 3,
  B3_GAP = 4, B3_TRANSFER = 5,
  B4_GAP = 6, B4_TRANSFER = 7
};

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

// ============================================================
// Belt geometry and planner constants
// These values are kept from arduino_refuge_circulation_test_2.ino.
// ============================================================
static const float BELT_LEN_MM[NUM_BELTS] = {500.0f, 1105.0f, 500.0f, 1105.0f};
static const float BELT_WIDTH_MM = 250.0f;
static const float CORNER_GAP_MM = 250.0f;
static const float SAFE_STEP_MM = 20.0f;
static const float STOP_EPS_MM = 1.0f;
static const float TOF_TOL_MM = 25.0f;
static const float POSITION_TOL_MM = 2.0f;

// Important: kept exactly from the changed logic file.
// B1=0, B2=1, B3=2, B4=3.
static const uint8_t LOAD_ORDER[NUM_BELTS] = {2, 1, 0, 3};

// ============================================================
// Encoder / motor control
// ============================================================
static float MM_PER_ENCODER_COUNT[NUM_BELTS] = {0.100f, 0.100f, 0.100f, 0.100f};

static const float ENCODER_PPR_FOR_RPM = 270.0f;

static int defaultPwm = 130;
static float defaultTargetRpm = 30.0f;

static float GLOBAL_KP = 1.5f;
static float GLOBAL_KI = 0.90f;
static float GLOBAL_KD = 0.00f;

static const int MIN_PWM = 35;
static const int MAX_PWM = 255;
static const int PWM_STEP_LIMIT = 50;
static const unsigned long PID_SAMPLE_TIME_MS = 100;

// ============================================================
// ToF filtering
// raw -> calibrated -> Kalman -> output deadband.
// Invalid/range fail still appears as 8190, matching the refuge code.
// ============================================================
static const uint32_t TOF_TIMING_BUDGET_US = 100000UL;
static const uint16_t TOF_TIMEOUT_MS = 500;
static const float TOF_SIGNAL_RATE_LIMIT = 1.20f;

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
static const float TOF_DEADBAND_MM = 3.0f;
static const uint16_t TOF_INVALID_MM = 8190;
static const int TOF_RAW_MIN_VALID_MM = 20;
static const int TOF_RAW_MAX_VALID_MM = 8000;

// ============================================================
// Runtime timing
// ============================================================
static const unsigned long SENSOR_PERIOD_MS = 50;
static const unsigned long STATUS_PERIOD_MS = 400;
static const unsigned long JAM_TIMEOUT_MS = 1200;
static const unsigned long MAX_MOVE_TIME_MS = 16000;

static const uint8_t MAX_BOX = 80;

// ============================================================
// Box database model
// ============================================================
struct Box {
  uint16_t id;
  uint16_t seq;
  uint8_t belt;
  float pos;
  float longSide;
  float shortSide;
  float height;
  bool active;
};

#endif