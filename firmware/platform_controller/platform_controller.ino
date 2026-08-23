#include "LiftStepper.h"
#include "PusherStepper.h"
#include "ServoController.h"

ServoController yawServo;
PusherStepper pusher;
LiftStepper lift;

const float YAW_TO_SERVO_GAIN = 1.0;
const int SERVO_CENTER_ANGLE = 90;
const bool AUTO_HOME_PUSHER_ON_START = false;
const int BARRIER_COUNT = 3;
const int BARRIER_SERVO_PINS[BARRIER_COUNT] = {9, 11, 12};
const int BARRIER_DOWN_ANGLE[BARRIER_COUNT] = {10, 5, 0};
const int BARRIER_UP_ANGLE[BARRIER_COUNT] = {90, 85, 80};
const int BARRIER_MIN_PULSE = 500;
const int BARRIER_MAX_PULSE = 2500;
Servo barrierServos[BARRIER_COUNT];
int barrierAngle[BARRIER_COUNT] = {0, 0, 0};
const int UNLOAD_PLATE_SERVO_PIN = 8;
const int UNLOAD_PLATE_DOWN_ANGLE = 40;
const int UNLOAD_PLATE_UP_ANGLE = 90;
const int UNLOAD_PLATE_STEP_DELAY_MS = 25;
Servo unloadPlateServo;
int unloadPlateAngle = UNLOAD_PLATE_DOWN_ANGLE;

String input = "";

int yawToServoAngle(float yawDeg) {
  int target = SERVO_CENTER_ANGLE + (int)(yawDeg * YAW_TO_SERVO_GAIN);
  return constrain(target, ServoController::MIN_ANGLE, ServoController::MAX_ANGLE);
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  S <angle>  : move servo angle, 0~180");
  Serial.println("  U <pulse>  : write raw servo pulse, 500~2500us");
  Serial.println("  C          : move servo to center");
  Serial.println("  Y <yaw>    : correct yaw, then return servo to center");
  Serial.println("  R <yaw>    : correct yaw, push 250mm, return 250mm");
  Serial.println("  P          : push 250mm, wait 1s, return 250mm");
  Serial.println("  PM <mm>    : move pusher to absolute mm from zero");
  Serial.println("  PR <mm>    : move pusher relative mm");
  Serial.println("  L U        : lift up 250mm");
  Serial.println("  L D        : lift down 250mm");
  Serial.println("  L 1/2/3    : lift to floor");
  Serial.println("  Z <mm>     : fine lift jog in mm, ex: Z 1 / Z -1");
  Serial.println("  Z0         : zero fine lift offset");
  Serial.println("  B <floor> UP/DOWN : move floor barrier, ex: B 1 UP");
  Serial.println("  B <floor> <angle> : move floor barrier angle, ex: B 2 80");
  Serial.println("  B UP/DOWN / B <angle> defaults to floor 1");
  Serial.println("  T UP/DOWN   : move unload plate servo");
  Serial.println("  T <angle>   : move unload plate servo angle, 0~180");
  Serial.println("  H          : zero pusher position");
  Serial.println("  ?          : print help");
  Serial.println();
}

int clampBarrierFloor(int floor) {
  if (floor < 1) return 1;
  if (floor > BARRIER_COUNT) return BARRIER_COUNT;
  return floor;
}

void moveBarrierTo(int floor, int angle) {
  floor = clampBarrierFloor(floor);
  int idx = floor - 1;
  barrierAngle[idx] = constrain(angle, 0, 180);
  if (!barrierServos[idx].attached()) {
    barrierServos[idx].attach(BARRIER_SERVO_PINS[idx], BARRIER_MIN_PULSE, BARRIER_MAX_PULSE);
  }
  barrierServos[idx].write(barrierAngle[idx]);
  delay(250);
  Serial.print("Barrier floor ");
  Serial.print(floor);
  Serial.print(" angle: ");
  Serial.println(barrierAngle[idx]);
}

void barrierUp(int floor) {
  floor = clampBarrierFloor(floor);
  moveBarrierTo(floor, BARRIER_UP_ANGLE[floor - 1]);
  Serial.print("Barrier floor ");
  Serial.print(floor);
  Serial.println(" up");
}

void barrierDown(int floor) {
  floor = clampBarrierFloor(floor);
  moveBarrierTo(floor, BARRIER_DOWN_ANGLE[floor - 1]);
  Serial.print("Barrier floor ");
  Serial.print(floor);
  Serial.println(" down");
}

void moveUnloadPlateTo(int angle) {
  int targetAngle = constrain(angle, 0, 180);
  if (!unloadPlateServo.attached()) {
    unloadPlateServo.attach(UNLOAD_PLATE_SERVO_PIN, BARRIER_MIN_PULSE, BARRIER_MAX_PULSE);
  }
  if (unloadPlateAngle == targetAngle) {
    unloadPlateServo.write(unloadPlateAngle);
    delay(250);
  } else {
    int step = (targetAngle >= unloadPlateAngle) ? 1 : -1;
    while (unloadPlateAngle != targetAngle) {
      unloadPlateAngle += step;
      unloadPlateServo.write(unloadPlateAngle);
      delay(UNLOAD_PLATE_STEP_DELAY_MS);
    }
  }
  Serial.print("Unload plate angle: ");
  Serial.println(unloadPlateAngle);
}

void unloadPlateUp() {
  moveUnloadPlateTo(UNLOAD_PLATE_UP_ANGLE);
  Serial.println("Unload plate up");
}

void unloadPlateDown() {
  moveUnloadPlateTo(UNLOAD_PLATE_DOWN_ANGLE);
  Serial.println("Unload plate down");
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(200);
  delay(1000);

  yawServo.begin();
  for (int floor = 1; floor <= BARRIER_COUNT; floor++) {
    int idx = floor - 1;
    barrierServos[idx].attach(BARRIER_SERVO_PINS[idx], BARRIER_MIN_PULSE, BARRIER_MAX_PULSE);
    barrierDown(floor);
  }
  unloadPlateServo.attach(UNLOAD_PLATE_SERVO_PIN, BARRIER_MIN_PULSE, BARRIER_MAX_PULSE);
  unloadPlateDown();
  pusher.begin();
  lift.begin();

  Serial.println("Platform controller ready");
  if (AUTO_HOME_PUSHER_ON_START) {
    Serial.println("Zeroing pusher...");
    pusher.home();
    Serial.println("Pusher zero done");
  } else {
    Serial.println("Pusher auto-zero skipped");
  }

  printHelp();
}

void loop() {
  lift.update();

  readSerialCommand();
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

  if (lower.startsWith("b")) {
    String arg = lower.substring(1);
    arg.trim();
    int floor = 1;
    String action = arg;
    int spaceIdx = arg.indexOf(' ');
    if (spaceIdx > 0) {
      String first = arg.substring(0, spaceIdx);
      String rest = arg.substring(spaceIdx + 1);
      first.trim();
      rest.trim();
      int requestedFloor = first.toInt();
      if (requestedFloor >= 1 && requestedFloor <= BARRIER_COUNT && rest.length() > 0) {
        floor = requestedFloor;
        action = rest;
      }
    }
    if (action == "up" || action == "u" || action == "+") {
      barrierUp(floor);
    } else if (action == "down" || action == "d" || action == "-") {
      barrierDown(floor);
    } else {
      moveBarrierTo(floor, action.toInt());
    }
  } else if (lower.startsWith("s")) {
    int angle = cmd.substring(1).toInt();
    yawServo.moveTo(angle);

    Serial.print("Servo angle: ");
    Serial.println(yawServo.currentAngle());
    Serial.print("Servo pulse: ");
    Serial.println(yawServo.currentPulse());
  } else if (lower.startsWith("u")) {
    int pulse = cmd.substring(1).toInt();
    yawServo.writePulse(pulse);

    Serial.print("Servo raw pulse: ");
    Serial.println(yawServo.currentPulse());
  } else if (lower.startsWith("t")) {
    String action = lower.substring(1);
    action.trim();
    if (action == "up" || action == "u" || action == "+") {
      unloadPlateUp();
    } else if (action == "down" || action == "d" || action == "-") {
      unloadPlateDown();
    } else {
      int angle = cmd.substring(1).toInt();
      moveUnloadPlateTo(angle);
    }
  } else if (lower.startsWith("y")) {
    float yaw = cmd.substring(1).toFloat();
    int angle = yawToServoAngle(yaw);

    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" -> servo angle: ");
    Serial.println(angle);

    yawServo.moveTo(angle);
    yawServo.moveToCenter();

    Serial.println("Servo returned to center");
  } else if (lower.startsWith("r")) {
    float yaw = cmd.substring(1).toFloat();
    int angle = yawToServoAngle(yaw);

    Serial.print("Run yaw correction: ");
    Serial.print(yaw);
    Serial.print(" -> servo angle: ");
    Serial.println(angle);

    yawServo.moveTo(angle);
    pusher.pushOnce();

    Serial.println("Run done");
  } else if (lower.startsWith("pm")) {
    float targetMm = cmd.substring(2).toFloat();
    pusher.moveToMm(targetMm);
  } else if (lower.startsWith("pr")) {
    float deltaMm = cmd.substring(2).toFloat();
    pusher.moveRelativeMm(deltaMm);
  } else if (lower == "p") {
    pusher.pushOnce();
    Serial.println("Push done");
  } else if (lower == "z0" || lower == "z 0" || lower == "zh") {
    lift.zeroOffset();
  } else if (lower.startsWith("z")) {
    float deltaMm = cmd.substring(1).toFloat();
    lift.jogMm(deltaMm);
  } else if (lower == "l u" || lower == "lu" || lower == "l +") {
      lift.moveUp();
  } else if (lower == "l d" || lower == "ld" || lower == "l -") {
      lift.moveDown();
  } else if (lower.startsWith("l")) {
      int floor = lower.substring(1).toInt();
      if (floor < 1 || floor > 3) {
        Serial.println("Unknown lift floor. Use: L 1 / L 2 / L 3");
      } else {
        lift.moveToFloor(floor);
      }
  } else if (lower == "c") {
    yawServo.moveToCenter();
    Serial.println("Servo centered");
  } else if (lower == "h") {
    pusher.home();
    Serial.println("Home done");
  } else if (lower == "?") {
    printHelp();
  } else {
    Serial.println("Unknown command");
    printHelp();
  }
}
