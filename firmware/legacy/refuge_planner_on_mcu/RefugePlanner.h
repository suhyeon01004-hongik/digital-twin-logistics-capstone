#ifndef REFUGE_PLANNER_H
#define REFUGE_PLANNER_H

// ============================================================
// Refuge circulation planner.
// Algorithm base is kept from arduino_refuge_circulation_test_2.ino.
//
// FIX:
// 1. Target belt movement has priority once receiver gap is ready.
// 2. Existing outbound handoff completion is only used when target cannot move.
// 3. Active outbound transfer still ignores receiver gap ToF after handoff starts,
//    because the passing box can naturally block the receiver gap sensor.
// ============================================================

void startAuto(uint16_t id) {
  if (waitingManualRefuge) {
    Serial.println(F("ERR REFUGE_PENDING use REFUGED or STOP"));
    return;
  }

  clearFault();

  int idx = findBox(id);
  if (idx < 0) {
    Serial.println(F("ERR TARGET_NOT_FOUND"));
    return;
  }

  targetId = id;
  completeTargetId = 0;
  autoMode = true;

  Serial.print(F("OK START P"));
  Serial.println(targetId);
}

void autoStep() {
  int targetIdx = findBox(targetId);

  if (targetIdx < 0) {
    setFault("TARGET_LOST");
    return;
  }

  if (targetAtUnloadZone(targetIdx)) {
    completeTargetId = targetId;
    autoMode = false;
    stopAllMotors();

    Serial.print(F("DONE TARGET_AT_UNLOAD_ZONE P"));
    Serial.println(targetId);
    return;
  }

  targetIdx = findBox(targetId);

  if (targetIdx < 0) {
    setFault("TARGET_LOST");
    return;
  }

  int targetBelt = boxes[targetIdx].belt;
  int neededGapBelt = nextBelt(targetBelt);

  // ============================================================
  // 가장 중요:
  // 타겟이 들어갈 다음 벨트 gap이 이미 준비되어 있으면,
  // 기존 outbound completion보다 target belt를 먼저 구동해야 함.
  //
  // 이전 문제:
  // B3 gap이 250 이상 만들어졌는데도 tryOutboundCompletion()이 먼저 실행되어
  // B3만 계속 돌아감.
  // ============================================================
  if (tryMoveTargetBelt(targetIdx, targetBelt, neededGapBelt)) return;

  // target을 아직 움직일 수 없을 때만,
  // 이미 시작된 handoff를 마저 넘김.
  if (tryOutboundCompletion()) return;

  targetIdx = findBox(targetId);

  if (targetIdx < 0) {
    setFault("TARGET_LOST");
    return;
  }

  targetBelt = boxes[targetIdx].belt;
  neededGapBelt = nextBelt(targetBelt);

  if (tryGapCreation(neededGapBelt, targetBelt)) return;
  if (tryCompact(neededGapBelt)) return;
  if (tryGreedySafeMove(targetIdx)) return;
  if (tryRefugeAction(targetIdx)) return;

  printAutoLockReport(targetIdx, targetBelt, neededGapBelt);
  setFault("AUTO_LOCK");
}

bool tryMoveTargetBelt(int targetIdx, int targetBelt, int neededGapBelt) {
  if (targetIdx < 0 || targetBelt < 0) return false;

  if (movingBeltWouldRotateInbound(targetBelt)) {
    saveAutoReasonTargetBlocked(targetBelt, neededGapBelt, "ROTATE_INBOUND");
    return false;
  }

  // 새 handoff를 시작하기 전에는 receiver gap이 준비되어야 함.
  // 단, target belt에서 이미 handoff가 시작된 상태라면
  // receiver gap ToF가 박스에 가려지는 것이 정상이므로 계속 진행 허용.
  bool activeOutbound = beltHasActiveOutboundTransfer(targetBelt);

  if (!activeOutbound && !topGapReady(neededGapBelt)) {
    saveAutoReasonTargetBlocked(targetBelt, neededGapBelt, "RECEIVER_NOT_READY");
    return false;
  }

  float d = safeForwardDistance(targetBelt, SAFE_STEP_MM, true);

  if (d > STOP_EPS_MM) {
    printAutoMove(F("AUTO_MOVE_TARGET"), targetBelt, 1, d);
    return runBeltMm(targetBelt, 1, d);
  }

  saveAutoReasonTargetBlocked(targetBelt, neededGapBelt, "SAFE_DISTANCE_ZERO");
  return false;
}

void saveAutoReasonTargetBlocked(int targetBelt, int neededGapBelt, const char* reason) {
  snprintf(
    lastAutoReason,
    sizeof(lastAutoReason),
    "target B%d need B%d blocked %s ready=%d rot=%d",
    targetBelt + 1,
    neededGapBelt + 1,
    reason,
    topGapReady(neededGapBelt) ? 1 : 0,
    movingBeltWouldRotateInbound(targetBelt) ? 1 : 0
  );
}

void printAutoMove(const __FlashStringHelper* label, int belt, int dir, float mm) {
  if (!debugAuto) return;

  Serial.print(label);
  Serial.print(F(" B"));
  Serial.print(belt + 1);

  if (dir > 0) Serial.print(F("+ "));
  else Serial.print(F("- "));

  Serial.print(mm, 1);
  Serial.print(F("mm next=B"));
  Serial.print(nextBelt(belt) + 1);

  Serial.print(F(" nextGapDb="));
  Serial.print(topGapDb(nextBelt(belt)), 1);

  Serial.print(F(" nextReady="));
  Serial.print(topGapReady(nextBelt(belt)) ? 1 : 0);

  Serial.println();
}

void printAutoCompact(int belt) {
  if (!debugAuto) return;

  Serial.print(F("AUTO_COMPACT B"));
  Serial.print(belt + 1);

  Serial.print(F(" total="));
  Serial.print(beltTotalAxisLength(belt), 1);

  Serial.print(F(" gapDb="));
  Serial.println(topGapDb(belt), 1);
}

void printMoveFault(const char* reason, int belt, int dir, float requested, float traveled) {
  Serial.print(F("MOVE_FAIL "));
  Serial.print(reason);

  Serial.print(F(" B"));
  Serial.print(belt + 1);

  if (dir > 0) Serial.print(F("+ requested="));
  else Serial.print(F("- requested="));

  Serial.print(requested, 1);

  Serial.print(F(" traveled="));
  Serial.println(traveled, 1);
}

void printAutoLockReport(int targetIdx, int targetBelt, int neededGapBelt) {
  Serial.println(F("LOCK_BEGIN"));

  if (targetIdx >= 0) {
    Serial.print(F("LOCK_TARGET P"));
    Serial.print(boxes[targetIdx].id);

    Serial.print(F(" B"));
    Serial.print(targetBelt + 1);

    Serial.print(F(" pos="));
    Serial.print(boxes[targetIdx].pos, 1);

    Serial.print(F(" axis="));
    Serial.print(axisLength(targetBelt, boxes[targetIdx]), 1);

    Serial.print(F(" needGapB"));
    Serial.println(neededGapBelt + 1);
  }

  Serial.print(F("LOCK_LAST "));
  Serial.println(lastAutoReason);

  if (targetBelt >= 0 && movingBeltWouldRotateInbound(targetBelt)) {
    Serial.println(F("LOCK_CAUSE target belt blocked by inbound overhang rotation risk"));
  }

  if (neededGapBelt >= 0 && !topGapReady(neededGapBelt)) {
    Serial.print(F("LOCK_CAUSE receiver gap not ready B"));
    Serial.print(neededGapBelt + 1);

    Serial.print(F(" db="));
    Serial.print(topGapDb(neededGapBelt), 1);

    if (useTof) {
      int ch = gapTofIndex(neededGapBelt);

      Serial.print(F(" tof="));
      Serial.print(tofMm[ch]);

      Serial.print(F(" tofOk="));
      Serial.print(tofOk[ch] ? 1 : 0);
    }

    Serial.println();
  }

  for (int b = 0; b < NUM_BELTS; b++) {
    printBeltDiag(b);
  }

  Serial.println(F("LOCK_END"));
}

void printBeltDiag(int belt) {
  Serial.print(F("LOCK_BELT B"));
  Serial.print(belt + 1);

  Serial.print(F(" count="));
  Serial.print(beltPackageCount(belt));

  Serial.print(F(" total="));
  Serial.print(beltTotalAxisLength(belt), 1);

  Serial.print(F(" gapDb="));
  Serial.print(topGapDb(belt), 1);

  Serial.print(F(" ready="));
  Serial.print(topGapReady(belt) ? 1 : 0);

  if (useTof) {
    int ch = gapTofIndex(belt);

    Serial.print(F(" tof="));
    Serial.print(tofMm[ch]);

    Serial.print(F(" tofOk="));
    Serial.print(tofOk[ch] ? 1 : 0);
  }

  Serial.print(F(" safeFwd="));
  Serial.print(safeForwardDistance(belt, SAFE_STEP_MM, true), 1);

  Serial.print(F(" noHandoff="));
  Serial.print(safeForwardDistance(belt, SAFE_STEP_MM, false), 1);

  Serial.print(F(" rotIn="));
  Serial.print(movingBeltWouldRotateInbound(belt) ? 1 : 0);

  int top = topPackageOnBelt(belt);
  int bottom = bottomPackageOnBelt(belt);

  if (top >= 0) {
    float len = axisLength(belt, boxes[top]);

    Serial.print(F(" top=P"));
    Serial.print(boxes[top].id);

    Serial.print(F("["));
    Serial.print(boxes[top].pos - len / 2.0f, 1);
    Serial.print(F(".."));
    Serial.print(boxes[top].pos + len / 2.0f, 1);
    Serial.print(F("]"));
  }

  if (bottom >= 0 && bottom != top) {
    float len = axisLength(belt, boxes[bottom]);

    Serial.print(F(" bottom=P"));
    Serial.print(boxes[bottom].id);

    Serial.print(F("["));
    Serial.print(boxes[bottom].pos - len / 2.0f, 1);
    Serial.print(F(".."));
    Serial.print(boxes[bottom].pos + len / 2.0f, 1);
    Serial.print(F("]"));
  }

  Serial.println();
}

bool tryOutboundCompletion() {
  float bestNeed = 100000.0f;
  int bestBelt = -1;

  for (int b = 0; b < NUM_BELTS; b++) {
    for (int i = 0; i < MAX_BOX; i++) {
      if (!boxes[i].active || boxes[i].belt != b) continue;

      float len = axisLength(b, boxes[i]);
      float tail = boxes[i].pos - len / 2.0f;
      float front = boxes[i].pos + len / 2.0f;

      if (
        front > BELT_LEN_MM[b] + POSITION_TOL_MM &&
        tail < BELT_LEN_MM[b] - POSITION_TOL_MM &&
        !movingBeltWouldRotateInbound(b)
      ) {
        // 이미 handoff가 시작된 박스는 receiver gap ToF를 다시 보지 않는다.
        // 넘어가는 박스가 다음 벨트 gap 센서를 가리는 것은 정상이다.
        float need = BELT_LEN_MM[b] - tail + POSITION_TOL_MM;

        if (need < bestNeed) {
          bestNeed = need;
          bestBelt = b;
        }
      }
    }
  }

  if (bestBelt >= 0) {
    float d = min(SAFE_STEP_MM, bestNeed);

    printAutoMove(F("AUTO_OUTBOUND_COMPLETE"), bestBelt, 1, d);

    return runBeltMm(bestBelt, 1, d);
  }

  return false;
}

bool tryGapCreation(int neededGapBelt, int targetBelt) {
  for (int k = 0; k < NUM_BELTS; k++) {
    int b = beltAfter(neededGapBelt, k);

    if (b == targetBelt) continue;
    if (movingBeltWouldRotateInbound(b)) continue;

    float d = safeForwardDistance(b, SAFE_STEP_MM, true);

    if (d > STOP_EPS_MM) {
      printAutoMove(F("AUTO_GAP_CREATE"), b, 1, d);

      if (runBeltMm(b, 1, d)) return true;
    }
  }

  for (int gapBelt = 0; gapBelt < NUM_BELTS; gapBelt++) {
    if (!topGapReady(gapBelt)) continue;

    int source = prevBelt(gapBelt);

    if (movingBeltWouldRotateInbound(source)) continue;

    float d = safeForwardDistance(source, SAFE_STEP_MM, true);

    if (d > STOP_EPS_MM) {
      printAutoMove(F("AUTO_GAP_CHASE"), source, 1, d);

      if (runBeltMm(source, 1, d)) return true;
    }
  }

  return false;
}

bool tryCompact(int neededGapBelt) {
  for (int k = 0; k < NUM_BELTS; k++) {
    int b = beltBefore(neededGapBelt, k);

    if (
      canCompactToFullTopGap(b) &&
      topGapDb(b) < CORNER_GAP_MM - POSITION_TOL_MM
    ) {
      printAutoCompact(b);
      compactBelt(b);
      return true;
    }
  }

  return false;
}

bool tryGreedySafeMove(int targetIdx) {
  if (targetIdx < 0) return false;

  int targetBelt = boxes[targetIdx].belt;

  int bestBelt = -1;
  float bestD = 0.0f;
  float bestScore = -100000.0f;

  for (int b = 0; b < NUM_BELTS; b++) {
    if (movingBeltWouldRotateInbound(b)) continue;

    float d = safeForwardDistance(b, SAFE_STEP_MM, true);

    if (d <= STOP_EPS_MM) continue;

    float score = d;

    if (b == targetBelt) score += 1000.0f;

    score += min(topGapDb(nextBelt(b)), CORNER_GAP_MM) * 0.5f;
    score += min(topGapDb(b), CORNER_GAP_MM) * 0.2f;

    if (b == 3) score += 1.0f;

    if (score > bestScore) {
      bestScore = score;
      bestBelt = b;
      bestD = d;
    }
  }

  if (bestBelt >= 0) {
    printAutoMove(F("AUTO_GREEDY_SAFE"), bestBelt, 1, bestD);

    return runBeltMm(bestBelt, 1, bestD);
  }

  return false;
}

bool tryRefugeAction(int targetIdx) {
  int candidate = chooseRefugeCandidate(targetIdx);

  if (candidate < 0) return false;

  if (!autoRefugeDrop) {
    requestManualRefuge(candidate);
    return true;
  }

  return tryRefugeDrop(candidate);
}

bool tryRefugeDrop(int candidate) {
  if (candidate < 0) return false;

  float len = axisLength(3, boxes[candidate]);
  float front = boxes[candidate].pos + len / 2.0f;

  if (!singleBoxDropSafe(candidate, front)) return false;

  Serial.print(F("REFUGE_AUTO_DROP P"));
  Serial.println(boxes[candidate].id);

  if (!runBeltMm(3, -1, front)) return false;

  if (removeBoxById(boxes[candidate].id, "REFUGE_AUTO_DB_REMOVE")) {
    refugeCount++;
  }

  if (!runBeltMm(3, 1, front)) return false;

  return true;
}

int chooseRefugeCandidate(int targetIdx) {
  if (targetIdx < 0) return -1;

  if (boxes[targetIdx].belt == 3 && targetHasB4BlockerAhead(targetIdx)) {
    return topB4PackageAheadOfTarget(targetIdx);
  }

  if (
    nextBelt(boxes[targetIdx].belt) == 3 &&
    topGapDb(3) < CORNER_GAP_MM - POSITION_TOL_MM
  ) {
    int top = topPackageOnBelt(3);

    if (top >= 0 && boxes[top].id != targetId) return top;
  }

  if (topGapDb(3) < CORNER_GAP_MM - POSITION_TOL_MM) {
    int top = topPackageOnBelt(3);

    if (top >= 0 && boxes[top].id != targetId) return top;
  }

  return -1;
}

void requestManualRefuge(int candidate) {
  if (candidate < 0 || !boxes[candidate].active) return;

  waitingManualRefuge = true;
  pendingRefugeId = boxes[candidate].id;
  autoMode = false;

  stopAllMotors();

  float len = axisLength(boxes[candidate].belt, boxes[candidate]);

  Serial.print(F("REFUGE_REQUEST P"));
  Serial.print(pendingRefugeId);

  Serial.print(F(" B"));
  Serial.print(boxes[candidate].belt + 1);

  Serial.print(F(" range="));
  Serial.print(boxes[candidate].pos - len / 2.0f, 1);
  Serial.print(F(".."));
  Serial.print(boxes[candidate].pos + len / 2.0f, 1);

  Serial.println(F(" REMOVE_BY_HAND_THEN_TYPE_REFUGED"));
}

void completeManualRefuge() {
  if (!waitingManualRefuge || pendingRefugeId == 0) {
    Serial.println(F("ERR NO_REFUGE_PENDING"));
    return;
  }

  uint16_t removedId = pendingRefugeId;

  bool removed = removeBoxById(removedId, "REFUGE_MANUAL_DB_REMOVE");

  waitingManualRefuge = false;
  pendingRefugeId = 0;

  if (removed) {
    refugeCount++;
  }

  if (targetId > 0 && findBox(targetId) >= 0 && !faulted) {
    autoMode = true;

    Serial.print(F("OK REFUGE_RESUME TARGET P"));
    Serial.println(targetId);
  } else {
    autoMode = false;
    Serial.println(F("OK REFUGE_DONE_IDLE"));
  }
}

bool targetAtUnloadZone(int idx) {
  if (idx < 0 || !boxes[idx].active || boxes[idx].belt != 3) return false;

  float len = axisLength(3, boxes[idx]);
  float tail = boxes[idx].pos - len / 2.0f;
  float front = boxes[idx].pos + len / 2.0f;

  if (front > CORNER_GAP_MM + POSITION_TOL_MM) return false;

  if (!transferSensorComplete(3, boxes[idx])) return false;

  for (int i = 0; i < MAX_BOX; i++) {
    if (i == idx || !boxes[i].active || boxes[i].belt != 3) continue;

    float otherLen = axisLength(3, boxes[i]);
    float otherTail = boxes[i].pos - otherLen / 2.0f;

    if (otherTail < tail - POSITION_TOL_MM) return false;
  }

  return true;
}

bool transferSensorComplete(int belt, const Box& box) {
  if (!useTof) return true;

  int ch = transferTofIndex(belt);

  if (!tofOk[ch]) return true;

  float expected = BELT_WIDTH_MM - crossLength(belt, box);

  return abs((float)tofMm[ch] - expected) <= TOF_TOL_MM;
}

#endif