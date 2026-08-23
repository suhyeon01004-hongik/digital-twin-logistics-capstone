#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

// ============================================================
// Motion execution and DB position update.
// Planner movement logic is kept from arduino_refuge_circulation_test_2.ino.
// ============================================================

bool runBeltMm(int belt, int dir, float mm) {
  if (faulted) return false;
  if (belt < 0 || belt >= NUM_BELTS || mm <= 0.0f) return false;
  if (MM_PER_ENCODER_COUNT[belt] <= 0.0f) {
    setFault("BAD_ENCODER_SCALE");
    return false;
  }

  stopAllMotors();
  activeBelt = belt;
  activeDir = dir;
  long startCount;
  long lastCount;
  noInterrupts();
  startCount = encoderCount[belt];
  lastCount = startCount;
  interrupts();

  unsigned long startMs = millis();
  unsigned long lastChangeMs = startMs;
  setMotorRunning(belt, dir, defaultPwm);

  while (true) {
    updateMotorPi();
    if (USE_ESTOP && digitalRead(ESTOP_PIN) == LOW) {
      setFault("ESTOP");
      break;
    }
    unsigned long now = millis();
    if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
      lastSensorMs = now;
      readTofSensors();
    }
    long current;
    noInterrupts();
    current = encoderCount[belt];
    interrupts();
    if (current != lastCount) {
      lastCount = current;
      lastChangeMs = now;
    }
    float traveled = labs(current - startCount) * MM_PER_ENCODER_COUNT[belt];
    if (traveled >= mm) break;
    if (now - lastChangeMs > JAM_TIMEOUT_MS && now - startMs > 500) {
      printMoveFault("ENCODER_JAM", belt, dir, mm, traveled);
      setFault("ENCODER_JAM");
      break;
    }
    if (now - startMs > MAX_MOVE_TIME_MS) {
      printMoveFault("MOVE_TIMEOUT", belt, dir, mm, traveled);
      setFault("MOVE_TIMEOUT");
      break;
    }
  }

  long endCount;
  noInterrupts();
  endCount = encoderCount[belt];
  interrupts();
  float traveled = labs(endCount - startCount) * MM_PER_ENCODER_COUNT[belt];
  stopAllMotors();
  applyBeltMovementToDb(belt, dir * traveled);
  activeBelt = -1;
  return !faulted;
}

void applyBeltMovementToDb(int belt, float signedMm) {
  for (int i = 0; i < MAX_BOX; i++) {
    if (boxes[i].active && boxes[i].belt == belt) {
      boxes[i].pos += signedMm;
    }
  }
  if (signedMm > 0) {
    updateForwardTransfers(belt);
  }
}

void updateForwardTransfers(int belt) {
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    if (tail >= BELT_LEN_MM[belt] - POSITION_TOL_MM) {
      int nb = nextBelt(belt);
      boxes[i].belt = nb;
      boxes[i].pos = incomingEntryPosition(nb, boxes[i]);
    }
  }
}

bool beltHasActiveOutboundTransfer(int belt) {
  if (belt < 0 || belt >= NUM_BELTS) return false;

  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;

    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    float front = boxes[i].pos + len / 2.0f;

    // 이미 다음 벨트로 걸쳐 넘어가기 시작한 박스.
    // 이 상태에서는 receiving gap ToF가 박스에 가려지는 것이 정상이다.
    if (front > BELT_LEN_MM[belt] + POSITION_TOL_MM &&
        tail < BELT_LEN_MM[belt] - POSITION_TOL_MM) {
      return true;
    }
  }

  return false;
}

float activeOutboundCompletionDistance(int belt) {
  if (belt < 0 || belt >= NUM_BELTS) return 0.0f;

  float bestNeed = 100000.0f;
  bool found = false;

  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;

    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    float front = boxes[i].pos + len / 2.0f;

    if (front > BELT_LEN_MM[belt] + POSITION_TOL_MM &&
        tail < BELT_LEN_MM[belt] - POSITION_TOL_MM) {
      float need = BELT_LEN_MM[belt] - tail + POSITION_TOL_MM;
      if (need < bestNeed) {
        bestNeed = need;
        found = true;
      }
    }
  }

  if (!found) return 0.0f;
  return max(0.0f, bestNeed);
}

float safeForwardDistance(int belt, float desired, bool allowHandoff) {
  float d = desired;
  int nb = nextBelt(belt);

  bool activeOutbound = beltHasActiveOutboundTransfer(belt);
  bool receiverReadyForNewHandoff = topGapReady(nb);

  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;

    float len = axisLength(belt, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    float front = boxes[i].pos + len / 2.0f;

    bool thisBoxAlreadyOutbound =
      front > BELT_LEN_MM[belt] + POSITION_TOL_MM &&
      tail < BELT_LEN_MM[belt] - POSITION_TOL_MM;

    if (!allowHandoff) {
      d = min(d, max(0.0f, BELT_LEN_MM[belt] - front));
      continue;
    }

    // 이미 handoff가 시작된 박스는 다음 벨트 gap 센서를 다시 보지 않는다.
    // 이때 receiving gap ToF는 넘어가는 박스 때문에 작아지는 것이 정상이다.
    if (thisBoxAlreadyOutbound) {
      continue;
    }

    // 새 handoff를 시작하려는 박스는 여전히 receiving belt의 gap 준비가 필요하다.
    // 단, 이미 같은 source belt에서 outbound transfer가 진행 중이면
    // 두 번째 박스가 동시에 boundary를 넘지 못하게 제한한다.
    if (front + d > BELT_LEN_MM[belt] + POSITION_TOL_MM) {
      if (!receiverReadyForNewHandoff || activeOutbound) {
        d = min(d, max(0.0f, BELT_LEN_MM[belt] - front - 1.0f));
      }
    }
  }

  return max(0.0f, d);
}

bool movingBeltWouldRotateInbound(int belt) {
  int source = prevBelt(belt);
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != source) continue;
    float len = axisLength(source, boxes[i]);
    float tail = boxes[i].pos - len / 2.0f;
    float front = boxes[i].pos + len / 2.0f;
    if (front > BELT_LEN_MM[source] + POSITION_TOL_MM &&
        tail < BELT_LEN_MM[source] - POSITION_TOL_MM) {
      return true;
    }
  }
  return false;
}

bool topGapReady(int belt) {
  bool dbReady = topGapDb(belt) >= CORNER_GAP_MM - POSITION_TOL_MM;
  if (!dbReady) return false;
  if (!useTof) return true;
  int ch = gapTofIndex(belt);
  if (!tofOk[ch]) return true;
  return tofMm[ch] >= (uint16_t)(CORNER_GAP_MM - TOF_TOL_MM);
}

float topGapDb(int belt) {
  float gap = BELT_LEN_MM[belt];
  for (int i = 0; i < MAX_BOX; i++) {
    if (!boxes[i].active || boxes[i].belt != belt) continue;
    float len = axisLength(belt, boxes[i]);
    gap = min(gap, boxes[i].pos - len / 2.0f);
  }
  return max(0.0f, gap);
}

#endif