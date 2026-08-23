#ifndef BOX_DATABASE_H
#define BOX_DATABASE_H

// ============================================================
// Belt geometry and package database helpers.
// Logic is kept from arduino_refuge_circulation_test_2.ino.
// ============================================================

bool canCompactToFullTopGap(int belt) {
  if (beltPackageCount(belt) <= 0) return false;
  if (beltHasOverhang(belt)) return false;
  return beltTotalAxisLength(belt) <= BELT_LEN_MM[belt] - CORNER_GAP_MM + POSITION_TOL_MM;
}

void compactBelt(int belt) {
  float travelTop = compactTravelToTop(belt);
  if (travelTop > STOP_EPS_MM) {
    runBeltMm(belt, -1, travelTop);
  }
  setCompactTopDb(belt);
  float travelBottom = compactTravelToBottom(belt);
  if (travelBottom > STOP_EPS_MM) {
    runBeltMm(belt, 1, travelBottom);
  }
  setCompactBottomDb(belt);
}

float compactTravelToTop(int belt) {
  return guaranteedCompactTravel(belt);
}

void setCompactTopDb(int belt) {
  int order[MAX_BOX];
  int n = sortedBeltIndices(belt, order);
  float cursor = 0.0f;
  for (int k = 0; k < n; k++) {
    int i = order[k];
    float len = axisLength(belt, boxes[i]);
    boxes[i].pos = cursor + len / 2.0f;
    cursor += len;
  }
}

float compactTravelToBottom(int belt) {
  return guaranteedCompactTravel(belt);
}

float guaranteedCompactTravel(int belt) {
  return max(0.0f, BELT_LEN_MM[belt] - beltTotalAxisLength(belt));
}

void setCompactBottomDb(int belt) {
  int order[MAX_BOX];
  int n = sortedBeltIndices(belt, order);
  float cursor = BELT_LEN_MM[belt] - beltTotalAxisLength(belt);
  for (int k = 0; k < n; k++) {
    int i = order[k];
    float len = axisLength(belt, boxes[i]);
    boxes[i].pos = cursor + len / 2.0f;
    cursor += len;
  }
}

int sortedBeltIndices(int belt, int* order) {
  int n = 0;
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active && boxes[i].belt == belt) {
      order[n++] = i;
    }
  }
  for (int a = 0; a < n - 1; a++) {
    for (int b = a + 1; b < n; b++) {
      if (boxes[order[b]].pos < boxes[order[a]].pos) {
        int tmp = order[a];
        order[a] = order[b];
        order[b] = tmp;
      }
    }
  }
  return n;
}

bool targetHasB4BlockerAhead(int targetIdx) {
  if (targetIdx < 0 || boxes[targetIdx].belt != 3) return false;
  float targetLen = axisLength(3, boxes[targetIdx]);
  float targetTail = boxes[targetIdx].pos - targetLen / 2.0f;
  for (int i = 0; i < MAX_BOX; i++) {
    if (i == targetIdx || !boxes[i].active || boxes[i].belt != 3) continue;
    float len = axisLength(3, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    if (tail < targetTail - POSITION_TOL_MM) return true;
  }
  return false;
}

int topB4PackageAheadOfTarget(int targetIdx) {
  float targetLen = axisLength(3, boxes[targetIdx]);
  float targetTail = boxes[targetIdx].pos - targetLen / 2.0f;
  float bestTail = 100000.0f;
  int best = -1;
  for (int i = 0; i < MAX_BOX; i++) {
    if (i == targetIdx || !boxes[i].active || boxes[i].belt != 3) continue;
    float len = axisLength(3, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    if (tail < targetTail - POSITION_TOL_MM && tail < bestTail) {
      bestTail = tail;
      best = i;
    }
  }
  return best;
}

int topPackageOnBelt(int belt) {
  float bestTail = 100000.0f;
  int best = -1;
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    if (tail < bestTail) {
      bestTail = tail;
      best = i;
    }
  }
  return best;
}

int bottomPackageOnBelt(int belt) {
  float bestFront = -100000.0f;
  int best = -1;
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    float front = boxes[i].pos + len / 2.0f;
    if (front > bestFront) {
      bestFront = front;
      best = i;
    }
  }
  return best;
}

bool singleBoxDropSafe(int candidate, float travel) {
  if (candidate < 0 || travel <= 0.0f) return false;
  for (int i = 0; i < MAX_BOX; i++) {
    if (i == candidate || !boxes[i].active || boxes[i].belt != 3) continue;
    float len = axisLength(3, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    if (tail < travel - POSITION_TOL_MM) return false;
  }
  return true;
}

bool beltHasOverhang(int belt) {
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    float front = boxes[i].pos + len / 2.0f;
    if (tail < -POSITION_TOL_MM || front > BELT_LEN_MM[belt] + POSITION_TOL_MM) {
      return true;
    }
  }
  return false;
}

float beltTotalAxisLength(int belt) {
  float total = 0.0f;
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active && boxes[i].belt == belt) {
      total += axisLength(belt, boxes[i]);
    }
  }
  return total;
}

int beltPackageCount(int belt) {
  int n = 0;
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active && boxes[i].belt == belt) n++;
  }
  return n;
}

float axisLength(int belt, const Box& box) {
  return (belt == 0 || belt == 2) ? box.longSide : box.shortSide;
}

float crossLength(int belt, const Box& box) {
  return (belt == 0 || belt == 2) ? box.shortSide : box.longSide;
}

float incomingEntryPosition(int belt, const Box& box) {
  float entryAxis = axisLength(belt, box);
  return max(entryAxis / 2.0f, CORNER_GAP_MM - entryAxis / 2.0f);
}

float appendPosFor(int belt, float longSide, float shortSide) {
  float cursor = 0.0f;
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    cursor = max(cursor, boxes[i].pos + len / 2.0f);
  }
  Box temp;
  temp.longSide = longSide;
  temp.shortSide = shortSide;
  return cursor + axisLength(belt, temp) / 2.0f;
}

bool addSequenceBox(uint16_t id, float longSide, float shortSide, float height) {
  if (findBox(id) >= 0) {
    Serial.println(F("ERR SEQ duplicate id"));
    return false;
  }
  int belt = chooseSequenceBelt(longSide, shortSide);
  if (belt < 0) {
    Serial.println(F("ERR SEQ layout full"));
    return false;
  }

  Box temp;
  temp.longSide = longSide;
  temp.shortSide = shortSide;
  float axis = axisLength(belt, temp);
  float pos = loadReserveForBelt(belt) + axis / 2.0f;
  if (!addBox(id, belt, pos, longSide, shortSide, height)) {
    return false;
  }

  int idx = findBox(id);
  if (idx >= 0) {
    boxes[idx].seq = nextSeqOrder - 1;
  }
  rebuildSequenceLayout();

  Serial.print(F("OK SEQ P"));
  Serial.print(id);
  Serial.print(F(" B"));
  Serial.print(belt + 1);
  Serial.print(F(" pos="));
  int storedIdx = findBox(id);
  if (storedIdx >= 0) Serial.print(boxes[storedIdx].pos, 1);
  else Serial.print(pos, 1);
  Serial.print(F(" stage="));
  Serial.println(loadStageIndex + 1);
  return true;
}

int chooseSequenceBelt(float longSide, float shortSide) {
  while (loadStageIndex < 4) {
    int belt = LOAD_ORDER[loadStageIndex];
    if (sequenceBeltCanAccept(belt, longSide, shortSide)) {
      return belt;
    }
    loadStageIndex++;
  }
  return -1;
}

float sequenceEntryGapDb(int belt) {
  float reserve = loadReserveForBelt(belt);
  float firstTail = BELT_LEN_MM[belt];
  bool hasBox = false;

  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    if (tail < firstTail) firstTail = tail;
    hasBox = true;
  }

  if (!hasBox) {
    return max(0.0f, BELT_LEN_MM[belt] - reserve);
  }

  return max(0.0f, firstTail - reserve);
}

bool sequenceBeltCanAccept(int belt, float longSide, float shortSide) {
  Box temp;
  temp.longSide = longSide;
  temp.shortSide = shortSide;

  float reserve = loadReserveForBelt(belt);
  float totalNow = beltTotalAxisLength(belt);
  float newAxis = axisLength(belt, temp);

  // 물리적 총 적재 가능 길이 확인.
  // B4는 초기 적재에서 최하단 250mm를 계속 비워두므로 reserve만큼 제외한다.
  if (totalNow + newAxis > BELT_LEN_MM[belt] - reserve + POSITION_TOL_MM) {
    return false;
  }

  // 이미 박스가 있는 벨트에 새 박스가 코너를 넘어 들어오려면,
  // 현재 벨트 입구에 250mm 이상의 빈공간이 있어야 한다.
  // 이 조건이 없으면 4호+2호 이후 3번째 2호가 B3에 들어오는 것처럼
  // 실제로는 진입 불가능한 초기 DB가 생성된다.
  if (beltPackageCount(belt) > 0) {
    float entryGapNow = sequenceEntryGapDb(belt);
    if (entryGapNow < CORNER_GAP_MM - POSITION_TOL_MM) {
      return false;
    }
  }

  return true;
}

void rebuildSequenceLayout() {
  for (int oi = 0; oi < 4; oi++) {
    rebuildSequenceBelt(LOAD_ORDER[oi]);
  }
}

void rebuildSequenceBelt(int belt) {
  int order[MAX_BOX];
  int n = sortedBeltIndicesNewestFirst(belt, order);

  // 초기 적재 중에는 새 박스가 들어오는 입구 쪽에 빈공간이 남도록
  // 박스들을 벨트 끝단 방향으로 밀착시킨 DB 상태로 유지한다.
  // B4는 최하단 reserve 250mm를 포함해서 비워둘 수 있다.
  float total = beltTotalAxisLength(belt);
  float cursor = BELT_LEN_MM[belt] - total;
  float reserve = loadReserveForBelt(belt);

  if (cursor < reserve) cursor = reserve;

  for (int k = 0; k < n; k++) {
    int i = order[k];
    float len = axisLength(belt, boxes[i]);
    boxes[i].pos = cursor + len / 2.0f;
    cursor += len;
  }
}

float loadReserveForBelt(int belt) {
  return belt == 3 ? CORNER_GAP_MM : 0.0f;
}

int sortedBeltIndicesNewestFirst(int belt, int* order) {
  int n = 0;
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active && boxes[i].belt == belt) {
      order[n++] = i;
    }
  }
  for (int a = 0; a < n - 1; a++) {
    for (int b = a + 1; b < n; b++) {
      if (boxes[order[b]].seq > boxes[order[a]].seq) {
        int tmp = order[a];
        order[a] = order[b];
        order[b] = tmp;
      }
    }
  }
  return n;
}

bool addBox(uint16_t id, int belt, float pos, float longSide, float shortSide, float height) {
  if (findBox(id) >= 0) {
    Serial.println(F("ERR ADD duplicate id"));
    return false;
  }
  int slot = -1;
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    Serial.println(F("ERR DB_FULL"));
    return false;
  }
  Box temp;
  temp.id = id;
  temp.seq = nextSeqOrder++;
  temp.belt = belt;
  temp.pos = pos;
  temp.longSide = longSide;
  temp.shortSide = shortSide;
  temp.height = height;
  temp.active = true;
  float len = axisLength(belt, temp);
  if (pos - len / 2.0f < -POSITION_TOL_MM ||
      pos + len / 2.0f > BELT_LEN_MM[belt] + POSITION_TOL_MM) {
    Serial.println(F("ERR ADD outside belt"));
    return false;
  }
  boxes[slot] = temp;
  boxCount = activeBoxCount();
  return true;
}

int activeBoxCount() {
  int n = 0;
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active) n++;
  }
  return n;
}

int findBox(uint16_t id) {
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active && boxes[i].id == id) return i;
  }
  return -1;
}

bool removeBoxById(uint16_t id, const char* label) {
  int idx = findBox(id);
  if (idx < 0) {
    Serial.print(F("ERR "));
    Serial.print(label);
    Serial.print(F(" P"));
    Serial.print(id);
    Serial.println(F(" NOT_FOUND"));
    return false;
  }
  boxes[idx].active = false;
  boxes[idx].belt = 0;
  boxes[idx].pos = 0.0f;
  boxCount = activeBoxCount();
  Serial.print(F("OK "));
  Serial.print(label);
  Serial.print(F(" P"));
  Serial.print(id);
  Serial.print(F(" boxes="));
  Serial.println(boxCount);
  return true;
}

void clearDb() {
  autoMode = false;
  targetId = 0;
  completeTargetId = 0;
  refugeCount = 0;
  waitingManualRefuge = false;
  pendingRefugeId = 0;
  nextSeqId = 1;
  nextSeqOrder = 1;
  loadStageIndex = 0;
  for (int i = 0; i < MAX_BOX; i++) {
    boxes[i].active = false;
    boxes[i].id = 0;
    boxes[i].seq = 0;
  }
  boxCount = 0;
}

#endif