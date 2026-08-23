#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

// ============================================================
// Serial command parser.
// Core command behavior is kept from arduino_refuge_circulation_test_2.ino.
// BOX/BOXID are compatibility wrappers around the same addSequenceBox path.
// ============================================================

void readSerialCommand() {
  static char buf[128];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Newline / Carriage Return 둘 다 명령 종료로 처리.
    // Arduino IDE, Flask/pyserial, 다른 터미널 모두 대응.
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
      continue;
    }

    if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  toUpperInPlace(cmd);

  if (strcmp(cmd, "HELP") == 0) {
    printHelp();
  } else if (strcmp(cmd, "CLEAR") == 0) {
    clearDb();
    Serial.println(F("OK CLEAR"));
  } else if (strcmp(cmd, "SEQ") == 0 || strcmp(cmd, "SEQADD") == 0) {
    commandSeqAdd(false);
  } else if (strcmp(cmd, "SEQID") == 0) {
    commandSeqAdd(true);
  } else if (strcmp(cmd, "BOX") == 0 || strcmp(cmd, "SEQBOX") == 0) {
    commandBoxAdd(false);
  } else if (strcmp(cmd, "BOXID") == 0 || strcmp(cmd, "SEQBOXID") == 0) {
    commandBoxAdd(true);
  } else if (strcmp(cmd, "ADD") == 0) {
    commandAdd(false);
  } else if (strcmp(cmd, "ADDPOS") == 0) {
    commandAdd(true);
  } else if (strcmp(cmd, "DEL") == 0 || strcmp(cmd, "REMOVE") == 0) {
    commandRemove();
  } else if (strcmp(cmd, "LIST") == 0) {
    printDb();
  } else if (strcmp(cmd, "START") == 0 || strcmp(cmd, "TARGET") == 0) {
    char* idTok = strtok(NULL, " ");
    if (!idTok) {
      Serial.println(F("ERR START needs package id"));
      return;
    }
    startAuto((uint16_t)atoi(idTok));
  } else if (strcmp(cmd, "STOP") == 0) {
    autoMode = false;
    waitingManualRefuge = false;
    pendingRefugeId = 0;
    stopAllMotors();
    Serial.println(F("OK STOP"));
  } else if (strcmp(cmd, "REFUGED") == 0 || strcmp(cmd, "REFUGE_DONE") == 0) {
    completeManualRefuge();
  } else if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(cmd, "SENSORS") == 0) {
    readTofSensors();
    printSensors();
  } else if (strcmp(cmd, "DIAG") == 0 || strcmp(cmd, "DEBUG") == 0) {
    printManualDiag();
  } else if (strcmp(cmd, "ZERO") == 0) {
    zeroEncoders();
  } else if (strcmp(cmd, "MOVE") == 0) {
    commandMove();
  } else if (strcmp(cmd, "SET") == 0) {
    commandSet();
  } else {
    Serial.println(F("ERR UNKNOWN_COMMAND"));
  }
}

void commandSeqAdd(bool explicitId) {
  char* firstTok = strtok(NULL, " ");
  char* longTok = explicitId ? strtok(NULL, " ") : firstTok;
  char* shortTok = strtok(NULL, " ");
  char* heightTok = strtok(NULL, " ");
  if (!firstTok || !longTok || !shortTok) {
    Serial.println(explicitId ? F("ERR SEQID id long short [height]") :
                                F("ERR SEQ long short [height]"));
    return;
  }
  uint16_t id = explicitId ? (uint16_t)atoi(firstTok) : nextSeqId;
  float longSide = atof(longTok);
  float shortSide = atof(shortTok);
  float height = heightTok ? atof(heightTok) : 100.0f;
  if (shortSide > longSide) {
    float tmp = longSide;
    longSide = shortSide;
    shortSide = tmp;
  }
  if (id == 0 || longSide <= 0.0f || shortSide <= 0.0f || height <= 0.0f) {
    Serial.println(F("ERR SEQ range"));
    return;
  }
  if (addSequenceBox(id, longSide, shortSide, height)) {
    if (!explicitId) nextSeqId++;
  }
}

void commandBoxAdd(bool explicitId) {
  char* firstTok = strtok(NULL, " ");
  if (!firstTok) {
    Serial.println(explicitId ? F("ERR BOXID id type") : F("ERR BOX type"));
    return;
  }

  uint16_t id = explicitId ? (uint16_t)atoi(firstTok) : nextSeqId;
  char* typeTok = explicitId ? strtok(NULL, " ") : firstTok;
  if (!typeTok) {
    Serial.println(explicitId ? F("ERR BOXID id type") : F("ERR BOX type"));
    return;
  }

  int parcelNo = atoi(typeTok);
  float longSide, shortSide, height;
  if (!parcelTypeDimensions(parcelNo, &longSide, &shortSide, &height)) {
    Serial.println(F("ERR BOX type 1..5"));
    return;
  }

  if (addSequenceBox(id, longSide, shortSide, height)) {
    if (!explicitId) nextSeqId++;
    Serial.print(F("OK BOX type="));
    Serial.print(parcelNo);
    Serial.print(F(" id="));
    Serial.println(id);
  }
}

bool parcelTypeDimensions(int parcelNo, float* longSide, float* shortSide, float* height) {
  if (!longSide || !shortSide || !height) return false;

  switch (parcelNo) {
    case 1:
      *longSide = 110.0f;
      *shortSide = 95.0f;
      *height = 45.0f;
      return true;
    case 2:
      *longSide = 135.0f;
      *shortSide = 90.0f;
      *height = 75.0f;
      return true;
    case 3:
      *longSide = 170.0f;
      *shortSide = 125.0f;
      *height = 105.0f;
      return true;
    case 4:
      *longSide = 205.0f;
      *shortSide = 155.0f;
      *height = 140.0f;
      return true;
    case 5:
      *longSide = 240.0f;
      *shortSide = 190.0f;
      *height = 170.0f;
      return true;
    default:
      return false;
  }
}

void commandAdd(bool explicitPos) {
  char* idTok = strtok(NULL, " ");
  char* beltTok = strtok(NULL, " ");
  char* posTok = explicitPos ? strtok(NULL, " ") : NULL;
  char* longTok = strtok(NULL, " ");
  char* shortTok = strtok(NULL, " ");
  char* heightTok = strtok(NULL, " ");
  if (!idTok || !beltTok || !longTok || !shortTok) {
    Serial.println(explicitPos ? F("ERR ADDPOS id belt pos long short [height]") :
                                 F("ERR ADD id belt long short [height]"));
    return;
  }
  uint16_t id = (uint16_t)atoi(idTok);
  int beltNo = atoi(beltTok);
  int belt = beltNoToIndex(beltNo);
  if (belt < 0 || id == 0) {
    Serial.println(F("ERR ADD range"));
    return;
  }
  float longSide = atof(longTok);
  float shortSide = atof(shortTok);
  float height = heightTok ? atof(heightTok) : 100.0f;
  if (shortSide > longSide) {
    float tmp = longSide;
    longSide = shortSide;
    shortSide = tmp;
  }
  float pos = explicitPos ? atof(posTok) : appendPosFor(belt, longSide, shortSide);
  if (addBox(id, belt, pos, longSide, shortSide, height)) {
    Serial.print(F("OK ADD P"));
    Serial.print(id);
    Serial.print(F(" B"));
    Serial.print(belt + 1);
    Serial.print(F(" pos="));
    Serial.println(pos, 1);
  }
}

void commandMove() {
  char* beltTok = strtok(NULL, " ");
  char* dirTok = strtok(NULL, " ");
  char* mmTok = strtok(NULL, " ");
  if (!beltTok || !dirTok || !mmTok) {
    Serial.println(F("ERR MOVE belt dir mm"));
    return;
  }
  int belt = beltNoToIndex(atoi(beltTok));
  int dir = atoi(dirTok);
  float mm = atof(mmTok);
  if (belt < 0 || (dir != 1 && dir != -1) || mm <= 0) {
    Serial.println(F("ERR MOVE range"));
    return;
  }
  autoMode = false;
  clearFault();
  if (runBeltMm(belt, dir, mm)) {
    Serial.println(F("OK MOVE_DONE"));
  }
}

void commandSet() {
  char* key = strtok(NULL, " ");
  char* val = strtok(NULL, " ");
  if (!key || !val) {
    Serial.println(F("ERR SET key value"));
    return;
  }

  toUpperInPlace(key);

  if (strcmp(key, "PWM") == 0) {
    defaultPwm = constrain(atoi(val), 0, 255);
    Serial.println(F("OK SET PWM"));
  } else if (strcmp(key, "RPM") == 0) {
    defaultTargetRpm = constrain(atof(val), 1.0f, 200.0f);
    Serial.println(F("OK SET RPM"));
  } else if (strcmp(key, "KP") == 0) {
    GLOBAL_KP = atof(val);
    Serial.println(F("OK SET KP"));
  } else if (strcmp(key, "KI") == 0) {
    GLOBAL_KI = atof(val);
    Serial.println(F("OK SET KI"));
  } else if (strcmp(key, "KD") == 0) {
    GLOBAL_KD = atof(val);
    Serial.println(F("OK SET KD"));
  } else if (strcmp(key, "TOF") == 0) {
    useTof = atoi(val) != 0;
    Serial.println(F("OK SET TOF"));
  } else if (strcmp(key, "DEBUG") == 0) {
    debugAuto = atoi(val) != 0;
    Serial.println(debugAuto ? F("OK SET DEBUG 1") : F("OK SET DEBUG 0"));
  } else if (strcmp(key, "REFUGE") == 0) {
    toUpperInPlace(val);
    if (strcmp(val, "AUTO") == 0 || strcmp(val, "1") == 0) {
      autoRefugeDrop = true;
      Serial.println(F("OK SET REFUGE AUTO"));
    } else if (strcmp(val, "MANUAL") == 0 || strcmp(val, "0") == 0) {
      autoRefugeDrop = false;
      Serial.println(F("OK SET REFUGE MANUAL"));
    } else {
      Serial.println(F("ERR SET REFUGE AUTO|MANUAL"));
    }
  } else if (strcmp(key, "MMCOUNT") == 0) {
    char* beltTok = val;
    char* scaleTok = strtok(NULL, " ");
    if (!scaleTok) {
      Serial.println(F("ERR SET MMCOUNT belt value"));
      return;
    }
    int belt = beltNoToIndex(atoi(beltTok));
    if (belt < 0) {
      Serial.println(F("ERR SET MMCOUNT belt"));
      return;
    }
    MM_PER_ENCODER_COUNT[belt] = atof(scaleTok);
    Serial.println(F("OK SET MMCOUNT"));
  } else {
    Serial.println(F("ERR SET UNKNOWN"));
  }
}

void commandRemove() {
  char* idTok = strtok(NULL, " ");
  if (!idTok) {
    Serial.println(F("ERR REMOVE id"));
    return;
  }
  uint16_t id = (uint16_t)atoi(idTok);
  if (removeBoxById(id, "REMOVE")) {
    if (id == targetId) {
      targetId = 0;
      completeTargetId = 0;
      autoMode = false;
    }
  }
}

void toUpperInPlace(char* s) {
  for (; *s; s++) {
    if (*s >= 'a' && *s <= 'z') *s = *s - 'a' + 'A';
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  CLEAR"));
  Serial.println(F("  SEQ long short [height]"));
  Serial.println(F("  SEQID id long short [height]"));
  Serial.println(F("  BOX type"));
  Serial.println(F("  BOXID id type"));
  Serial.println(F("  ADD id belt long short [height]"));
  Serial.println(F("  ADDPOS id belt pos long short [height]"));
  Serial.println(F("  DEL id"));
  Serial.println(F("  LIST"));
  Serial.println(F("  START id"));
  Serial.println(F("  REFUGED"));
  Serial.println(F("  STOP"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  SENSORS"));
  Serial.println(F("  DIAG"));
  Serial.println(F("  ZERO"));
  Serial.println(F("  MOVE belt dir mm"));
  Serial.println(F("  SET PWM value"));
  Serial.println(F("  SET RPM value"));
  Serial.println(F("  SET KP value"));
  Serial.println(F("  SET KI value"));
  Serial.println(F("  SET KD value"));
  Serial.println(F("  SET TOF 0|1"));
  Serial.println(F("  SET DEBUG 0|1"));
  Serial.println(F("  SET REFUGE MANUAL|AUTO"));
  Serial.println(F("  SET MMCOUNT belt mm_per_count"));
}

void printStatus() {
  Serial.print(F("STATUS floor="));
  Serial.print(FLOOR_ID);
  Serial.print(F(" mode="));
  if (faulted) Serial.print(F("FAULT"));
  else if (waitingManualRefuge) Serial.print(F("WAIT_REFUGE"));
  else if (autoMode) Serial.print(F("AUTO"));
  else Serial.print(F("IDLE"));
  Serial.print(F(" boxes="));
  Serial.print(activeBoxCount());
  Serial.print(F(" target="));
  Serial.print(targetId);
  Serial.print(F(" complete="));
  Serial.print(completeTargetId);
  Serial.print(F(" refuge="));
  Serial.print(refugeCount);
  Serial.print(F(" refugeMode="));
  if (autoRefugeDrop) Serial.print(F("AUTO"));
  else Serial.print(F("MANUAL"));
  if (waitingManualRefuge) {
    Serial.print(F(" pendingRefuge=P"));
    Serial.print(pendingRefugeId);
  }
  if (activeBelt >= 0) {
    Serial.print(F(" moving=B"));
    Serial.print(activeBelt + 1);
    Serial.print(activeDir > 0 ? F("+") : F("-"));
  }
  if (faulted) {
    Serial.print(F(" fault="));
    Serial.print(faultText);
  }
  Serial.print(F(" debug="));
  Serial.print(debugAuto ? 1 : 0);
  if (lastAutoReason[0] != '\0') {
    Serial.print(F(" last=\""));
    Serial.print(lastAutoReason);
    Serial.print(F("\""));
  }
  Serial.println();
}

void printManualDiag() {
  int targetIdx = findBox(targetId);
  int targetBelt = -1;
  int neededGapBelt = 3;
  if (targetIdx >= 0) {
    targetBelt = boxes[targetIdx].belt;
    neededGapBelt = nextBelt(targetBelt);
  }
  printAutoLockReport(targetIdx, targetBelt, neededGapBelt);
}

void printSensors() {
  Serial.print(F("SENSORS"));
  for (int i = 0; i < 8; i++) {
    Serial.print(' ');
    Serial.print(TOF_NAME[i]);
    Serial.print('=');
    Serial.print(tofMm[i]);
  }
  Serial.println();
}

void printDb() {
  Serial.println(F("DB_BEGIN"));
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active) continue;
    Serial.print(F("P"));
    Serial.print(boxes[i].id);
    Serial.print(F(" seq="));
    Serial.print(boxes[i].seq);
    Serial.print(F(" B"));
    Serial.print(boxes[i].belt + 1);
    Serial.print(F(" pos="));
    Serial.print(boxes[i].pos, 1);
    Serial.print(F(" long="));
    Serial.print(boxes[i].longSide, 1);
    Serial.print(F(" short="));
    Serial.print(boxes[i].shortSide, 1);
    Serial.print(F(" axis="));
    Serial.println(axisLength(boxes[i].belt, boxes[i]), 1);
  }
  Serial.println(F("DB_END"));
}

#endif