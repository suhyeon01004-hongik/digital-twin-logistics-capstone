// NEMA17 + TB6600 Linear Pusher Test
// Control by relative linear distance in millimeters.
//
// Assumptions:
// - Step angle: 1.8 deg -> 200 full steps/rev
// - TB6600 microstep setting: 200 pulse/rev
// - Lead screw pitch/lead: 4 mm/rev
// - Therefore: 200 / 4 = 50 pulse/mm
//
// Commands:
//   move 10     : move +10 mm
//   move -10    : move -10 mm
//   pos         : print estimated position
//   zero        : set current position as 0 mm
//   stop        : deceleration stop
//   ?           : print help
//
// ENA not used.
// Pin 9 not used.

const int PUL_PIN = 5;
const int DIR_PIN = 6;

const bool DIR_POSITIVE = HIGH;
const bool DIR_NEGATIVE = LOW;

const float LEAD_MM_PER_REV = 4.0;
const long PULSES_PER_REV = 400L;
const float PULSES_PER_MM = PULSES_PER_REV / LEAD_MM_PER_REV;

const unsigned int PULSE_WIDTH_US = 10;

const unsigned long START_INTERVAL_US  = 1000;
const unsigned long TARGET_INTERVAL_US = 200;
const unsigned long RAMP_STEP_US = 10;

enum MoveState {
  IDLE,
  MOVING,
  DECEL_STOP
};

MoveState moveState = IDLE;

long currentPulsePosition = 0;
long targetPulsePosition = 0;
long remainingPulses = 0;

bool currentDir = DIR_POSITIVE;
unsigned long currentIntervalUs = START_INTERVAL_US;
unsigned long lastStepTime = 0;

String input = "";

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, currentDir);

  Serial.begin(9600);
  printHelp();
}

void loop() {
  readSerialCommand();
  updateStepper();
}

void printHelp() {
  Serial.println();
  Serial.println("NEMA17 + TB6600 Linear Pusher Test");
  Serial.println("Microstep: 200 pulse/rev");
  Serial.println("Lead screw: 4 mm/rev");
  Serial.println("Scale: 50 pulse/mm");
  Serial.println("Commands:");
  Serial.println("  move 10   : move +10 mm");
  Serial.println("  move -10  : move -10 mm");
  Serial.println("  pos       : print estimated position");
  Serial.println("  zero      : set current position as 0 mm");
  Serial.println("  stop      : deceleration stop");
  Serial.println("  ?         : print help");
  Serial.println();
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      input.trim();
      if (input.length() > 0) {
        processCommand(input);
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  String lower = cmd;
  lower.toLowerCase();

  if (lower.startsWith("move")) {
    String valueText = cmd.substring(4);
    valueText.trim();
    float distanceMm = valueText.toFloat();
    if (distanceMm == 0.0) {
      Serial.println("Move distance is 0 mm");
      return;
    }
    startRelativeMove(distanceMm);
  } else if (lower == "pos") {
    printPosition();
  } else if (lower == "zero") {
    currentPulsePosition = 0;
    targetPulsePosition = 0;
    Serial.println("Position zeroed");
    printPosition();
  } else if (lower == "stop" || lower == "0") {
    if (moveState == MOVING) {
      moveState = DECEL_STOP;
      Serial.println("Command: deceleration stop");
    } else {
      Serial.println("Motor already stopped");
    }
  } else if (lower == "?") {
    printHelp();
  } else {
    Serial.println("Unknown command. Use: move <mm> / pos / zero / stop / ?");
  }
}

void startRelativeMove(float distanceMm) {
  if (moveState != IDLE) {
    Serial.println("Busy. Send stop first.");
    return;
  }

  long pulses = lround(abs(distanceMm) * PULSES_PER_MM);
  if (pulses <= 0) {
    Serial.println("Distance is too small for current pulse/mm setting");
    return;
  }

  currentDir = (distanceMm >= 0.0) ? DIR_POSITIVE : DIR_NEGATIVE;
  digitalWrite(DIR_PIN, currentDir);

  remainingPulses = pulses;
  targetPulsePosition = currentPulsePosition + ((distanceMm >= 0.0) ? pulses : -pulses);
  currentIntervalUs = START_INTERVAL_US;
  lastStepTime = micros();
  moveState = MOVING;

  Serial.print("Move command: ");
  Serial.print(distanceMm, 3);
  Serial.print(" mm, pulses=");
  Serial.println(pulses);
}

void updateStepper() {
  if (moveState == IDLE) {
    return;
  }

  unsigned long now = micros();
  if (now - lastStepTime < currentIntervalUs) {
    return;
  }
  lastStepTime = now;

  if (moveState == MOVING && remainingPulses <= 0) {
    finishMove();
    return;
  }

  outputOnePulse();

  if (currentDir == DIR_POSITIVE) {
    currentPulsePosition++;
  } else {
    currentPulsePosition--;
  }

  if (moveState == MOVING) {
    remainingPulses--;
    updateMoveRamp();
    if (remainingPulses <= 0) {
      finishMove();
    }
  } else if (moveState == DECEL_STOP) {
    updateStopRamp();
  }
}

void outputOnePulse() {
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(PUL_PIN, LOW);
}

void updateMoveRamp() {
  long decelPulses = pulsesNeededToSlowDown();

  if (remainingPulses <= decelPulses) {
    if (currentIntervalUs < START_INTERVAL_US - RAMP_STEP_US) {
      currentIntervalUs += RAMP_STEP_US;
    } else {
      currentIntervalUs = START_INTERVAL_US;
    }
  } else if (currentIntervalUs > TARGET_INTERVAL_US + RAMP_STEP_US) {
    currentIntervalUs -= RAMP_STEP_US;
  } else {
    currentIntervalUs = TARGET_INTERVAL_US;
  }
}

void updateStopRamp() {
  if (currentIntervalUs < START_INTERVAL_US - RAMP_STEP_US) {
    currentIntervalUs += RAMP_STEP_US;
  } else {
    currentIntervalUs = START_INTERVAL_US;
    moveState = IDLE;
    remainingPulses = 0;
    targetPulsePosition = currentPulsePosition;
    digitalWrite(PUL_PIN, LOW);
    Serial.println("Motor stopped");
    printPosition();
  }
}

long pulsesNeededToSlowDown() {
  if (currentIntervalUs >= START_INTERVAL_US) {
    return 0;
  }
  return (long)((START_INTERVAL_US - currentIntervalUs) / RAMP_STEP_US);
}

void finishMove() {
  moveState = IDLE;
  remainingPulses = 0;
  currentIntervalUs = START_INTERVAL_US;
  digitalWrite(PUL_PIN, LOW);
  Serial.println("Move done");
  printPosition();
}

void printPosition() {
  Serial.print("Position: ");
  Serial.print(currentPulsePosition / PULSES_PER_MM, 3);
  Serial.print(" mm, pulses=");
  Serial.println(currentPulsePosition);
}
