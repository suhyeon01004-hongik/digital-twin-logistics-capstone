// NEMA17 + TB6600 Test Code with Accel/Decel
// Microstep: 200 pulse/rev
// go   : clockwise
// back : counter-clockwise
// 0    : deceleration stop
// ENA not used
// Pin 9 not used

const int PUL_PIN = 5;
const int DIR_PIN = 6;

const bool DIR_CW  = HIGH;
const bool DIR_CCW = LOW;

// TB6600 setting
const int PULSES_PER_REV = 800;

// Pulse width
const unsigned int PULSE_WIDTH_US = 10;

// Speed setting
// Smaller value is faster.
// Based on 200 pulse/rev:
// 1000 us ~= 300 rpm
// 800 us  ~= 375 rpm
// 400 us  ~= 750 rpm

const unsigned long START_INTERVAL_US  = 1000;  // start speed, larger is slower
const unsigned long TARGET_INTERVAL_US = 200;   // target speed, smaller is faster

// Accel/decel slope
// Smaller value is smoother and slower to accelerate/decelerate.
const unsigned long RAMP_STEP_US = 10;

enum MotorState {
  STOPPED,
  RUNNING,
  DECEL_TO_STOP,
  DECEL_TO_REVERSE
};

MotorState motorState = STOPPED;

bool currentDir = DIR_CW;
bool pendingDir = DIR_CW;

unsigned long currentIntervalUs = START_INTERVAL_US;
unsigned long lastStepTime = 0;

String input = "";

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, currentDir);

  Serial.begin(9600);
  Serial.println("NEMA17 + TB6600 Accel/Decel Test");
  Serial.println("Microstep: 200 pulse/rev");
  Serial.println("Input: go / back / 0");
}

void loop() {
  readSerialCommand();
  updateStepper();
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      input.trim();
      input.toLowerCase();

      if (input.length() > 0) {
        processCommand(input);
      }

      input = "";
    } else {
      input += c;

      // 0 command can stop without Enter.
      if (input == "0") {
        processCommand(input);
        input = "";
      }
    }
  }
}

void processCommand(String cmd) {
  if (cmd == "go") {
    startOrReverse(DIR_CW);
    Serial.println("Command: go");
  } else if (cmd == "back") {
    startOrReverse(DIR_CCW);
    Serial.println("Command: back");
  } else if (cmd == "0") {
    if (motorState != STOPPED) {
      motorState = DECEL_TO_STOP;
      Serial.println("Command: deceleration stop");
    } else {
      Serial.println("Motor already stopped");
    }
  } else {
    Serial.println("Unknown command. Use: go / back / 0");
  }
}

void startOrReverse(bool newDir) {
  // If stopped, start acceleration in the requested direction.
  if (motorState == STOPPED) {
    currentDir = newDir;
    digitalWrite(DIR_PIN, currentDir);

    currentIntervalUs = START_INTERVAL_US;
    lastStepTime = micros();

    motorState = RUNNING;
  }

  // If same direction, keep running/accelerating.
  else if (currentDir == newDir) {
    motorState = RUNNING;
  }

  // If reverse direction is requested, decelerate before direction change.
  else {
    pendingDir = newDir;
    motorState = DECEL_TO_REVERSE;
    Serial.println("Decelerating before reverse...");
  }
}

void updateStepper() {
  if (motorState == STOPPED) {
    return;
  }

  unsigned long now = micros();

  if (now - lastStepTime >= currentIntervalUs) {
    lastStepTime = now;

    // Output one pulse.
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PUL_PIN, LOW);

    updateRamp();
  }
}

void updateRamp() {
  if (motorState == RUNNING) {
    // Acceleration: decreasing interval increases speed.
    if (currentIntervalUs > TARGET_INTERVAL_US + RAMP_STEP_US) {
      currentIntervalUs -= RAMP_STEP_US;
    } else {
      currentIntervalUs = TARGET_INTERVAL_US;
    }
  } else if (motorState == DECEL_TO_STOP) {
    // Deceleration: increasing interval decreases speed.
    if (currentIntervalUs < START_INTERVAL_US - RAMP_STEP_US) {
      currentIntervalUs += RAMP_STEP_US;
    } else {
      currentIntervalUs = START_INTERVAL_US;
      motorState = STOPPED;
      digitalWrite(PUL_PIN, LOW);
      Serial.println("Motor stopped");
    }
  } else if (motorState == DECEL_TO_REVERSE) {
    // Decelerate before reversing direction.
    if (currentIntervalUs < START_INTERVAL_US - RAMP_STEP_US) {
      currentIntervalUs += RAMP_STEP_US;
    } else {
      // Change direction after slowing down enough.
      currentDir = pendingDir;
      digitalWrite(DIR_PIN, currentDir);

      currentIntervalUs = START_INTERVAL_US;
      motorState = RUNNING;

      Serial.println("Direction changed. Accelerating...");
    }
  }
}
