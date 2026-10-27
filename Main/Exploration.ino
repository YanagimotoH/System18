#include <Arduino.h>
#include "RobotConfig.h"

static int searchStep = 0; // 探索パターンの段階。2000以上は検出位置の方位を表す
static unsigned long searchStepStartTime = 0; // 現在の探索段階を開始した時刻
static bool searchInitialized = false; // 探索パターンを初期化済みか示すフラグ

static bool approachConfirming = false; // 0～3cmを維持できているか確認中
static unsigned long approachContactStartTime = 0; // 接触確認開始時刻。候補検出時は探索段階の退避にも使う
static uint8_t targetPulseSamples = 0; // 対象候補の反射を連続確認した回数
static int pulseTargetDistanceCm = -1; // 直近の反射距離。反射が途切れた場合の候補距離にも使う
static bool approachTemporarilyLost = false; // 検出位置へ戻って対象を再確認中か示す
static unsigned long approachLostStartTime = 0; // 最後に反射を受けた時の方位（度）を保持

// 探索段階と、探索・接近確認に使う状態を初期化する。
void resetSearchPattern() {
  searchStep = 8;
  searchStepStartTime = millis();
  searchInitialized = true;
  approachConfirming = false;
  approachTemporarilyLost = false;
  targetPulseSamples = 0;
}

// 距離の急変や短い反射を候補として保持し、連続サンプルで対象を絞り込む。
bool detectTarget() {
  const int d = sensors.distanceCm;
  if (d >= 0 && d <= TARGET_DETECTION_RANGE_CM) {
    if (targetPulseSamples == 0) {
      targetPulseSamples = 1;
      pulseTargetDistanceCm = d;
      approachLostStartTime = (unsigned long)(readHeadingDeg() + 0.5f);
      return false;
    } else if (abs(d - pulseTargetDistanceCm) <= 5) {
      if (targetPulseSamples < 9) ++targetPulseSamples;
      pulseTargetDistanceCm = d;
      approachLostStartTime = (unsigned long)(readHeadingDeg() + 0.5f);
      if (targetPulseSamples >= 2) {
        approachContactStartTime = (unsigned long)searchStep;
        // 接近前に元の探索段階を一時退避し、2000+方位角を検出位置として格納する。
        searchStep = 2000 + (int)approachLostStartTime % 360;
        targetPulseSamples = 0;
        return true;
      }
      return false;
    } else {
      targetPulseSamples = 1; // 距離が変わったため新しいカップ候補として確認し直す
      approachLostStartTime = (unsigned long)(readHeadingDeg() + 0.5f);
    }
    pulseTargetDistanceCm = d;
    return false;
  }

  // 短く続いた反射の消失はカップ候補として接近時に確認する。
  const bool singlePulse = targetPulseSamples > 0 && targetPulseSamples < 9;
  targetPulseSamples = 0;
  if (singlePulse) {
    approachContactStartTime = (unsigned long)searchStep;
    searchStep = 2000 + (int)approachLostStartTime % 360;
  }
  return singlePulse;
}

// 現在の測定値を優先し、反射が途切れた場合は最後の候補距離を返す。
int targetDistance() {
  return sensors.distanceCm >= 0 ? sensors.distanceCm : pulseTargetDistanceCm;
}

// 探索パターンに沿って走行し、対象候補を見つけたら接近状態へ移る。
void search() {
  if (sensors.targetDetected) {
    // 短く続いた反射候補を接近確認へ渡す。
    stopMotors();
    state = APPROACH;
    approachTemporarilyLost = false;
    return;
  }
  // 探索パターンの初期化を行う。
  if (!searchInitialized) {
    searchStep = 8;
    searchStepStartTime = millis();
    searchInitialized = true;
  }
  // 0/2/4/6は直線走行、100～306は前方150度確認、11～43はレーン切替。
  if (searchStep == 8) {
    if (turnToHeading(NORTH_HEADING_DEG)) {
      searchStep = 0;
      searchStepStartTime = millis();
    }
    return;
  }
  if (searchStep == 0 || searchStep == 2 ||
      searchStep == 4 || searchStep == 6) {
    moveForward(FORWARD_SPEED);
    if (millis() - searchStepStartTime >= 1000) {
      stopMotors();
      searchStep = 100 + searchStep;
    }
    return;
  }
  if (searchStep >= 100 && searchStep < 110) {
    if (turnToHeading(searchStep % 10 == 0 || searchStep % 10 == 4
                          ? 285.0f
                          : 105.0f)) {
      searchStep += 100;
    }
    return;
  }
  if (searchStep >= 200 && searchStep < 210) {
    if (turnToHeading(searchStep % 10 == 0 || searchStep % 10 == 4
                          ? 75.0f
                          : 255.0f)) {
      searchStep += 100;
    }
    return;
  }
  if (searchStep >= 300 && searchStep < 310) {
    if (turnToHeading(searchStep % 10 == 0 || searchStep % 10 == 4
                          ? NORTH_HEADING_DEG
                          : SOUTH_HEADING_DEG)) {
      searchStep %= 10;
      searchStepStartTime = millis();
    }
    return;
  }
  if (searchStep == 11 && turnToHeading(90.0f)) {
    searchStep = 12;
    searchStepStartTime = millis();
  } else if (searchStep == 12) {
    moveForward(FORWARD_SPEED);
    if (millis() - searchStepStartTime >= 800) {
      stopMotors();
      searchStep = 13;
    }
  } else if (searchStep == 13 && turnToHeading(SOUTH_HEADING_DEG)) {
    searchStep = 2;
    searchStepStartTime = millis();
  } else if (searchStep == 21 && turnToHeading(90.0f)) {
    searchStep = 22;
    searchStepStartTime = millis();
  } else if (searchStep == 22) {
    moveForward(FORWARD_SPEED);
    if (millis() - searchStepStartTime >= 800) {
      stopMotors();
      searchStep = 23;
    }
  } else if (searchStep == 23 && turnToHeading(NORTH_HEADING_DEG)) {
    searchStep = 0;
    searchStepStartTime = millis();
  } else if (searchStep == 31 && turnToHeading(270.0f)) {
    searchStep = 32;
    searchStepStartTime = millis();
  } else if (searchStep == 32) {
    moveForward(FORWARD_SPEED);
    if (millis() - searchStepStartTime >= 800) {
      stopMotors();
      searchStep = 33;
    }
  } else if (searchStep == 33 && turnToHeading(SOUTH_HEADING_DEG)) {
    searchStep = 6;
    searchStepStartTime = millis();
  } else if (searchStep == 41 && turnToHeading(270.0f)) {
    searchStep = 42;
    searchStepStartTime = millis();
  } else if (searchStep == 42) {
    moveForward(FORWARD_SPEED);
    if (millis() - searchStepStartTime >= 800) {
      stopMotors();
      searchStep = 43;
    }
  } else if (searchStep == 43 && turnToHeading(NORTH_HEADING_DEG)) {
    searchStep = 4;
    searchStepStartTime = millis();
  }
}

// 検出した方位へ戻って候補を再確認し、近距離で確認後に運搬状態へ移る。
void approachTarget() {
  int currentDistanceCm = sensors.distanceCm;

  // カップを検出した瞬間の方位に戻してから、距離を確認する。
  if (!approachTemporarilyLost) {
    approachTemporarilyLost = true;
    stopMotors();
    return;
  }
  if (searchStep < 2000 || searchStep >= 2360 ||
      !turnToHeading((float)(searchStep - 2000))) {
    return;
  }

  if (approachTemporarilyLost) {
    stopMotors();
    if (currentDistanceCm < 0 ||
        currentDistanceCm > TARGET_DETECTION_RANGE_CM) {
      approachTemporarilyLost = false;
      sensors.targetDetected = false;
      resetSearchPattern();
      state = SEARCH;
      return;
    }
    if (targetPulseSamples < 2) {
      if (targetPulseSamples == 0) {
        pulseTargetDistanceCm = currentDistanceCm;
        targetPulseSamples = 1;
        return;
      }
      if (abs(currentDistanceCm - pulseTargetDistanceCm) > 5) {
        pulseTargetDistanceCm = currentDistanceCm;
        return;
      }
      targetPulseSamples = 2;
    }
    sensors.targetDetected = true;
  }

  // 0～3cmを検出したら、その判定を保持して1秒前進し、CARRYへ移る。
  if (approachConfirming) {
    if (millis() - approachContactStartTime < 1000) {
      moveForward(FORWARD_SPEED);
      return;
    }

    approachConfirming = false;
    stopMotors();
    sensors.targetDetected = false;
    resetCarryMotion();
    state = CARRY;
    return;
  }

  if (currentDistanceCm >= 0 && currentDistanceCm <= 3) {
    approachConfirming = true;
    approachContactStartTime = millis();
    moveForward(FORWARD_SPEED);
    return;
  }

  if (!sensors.targetDetected) {
    approachConfirming = false;
    state = SEARCH;
    return;
  }

  moveForward(FORWARD_SPEED);
}

// 停止・後退・旋回を非ブロッキングで進め、回避前の状態に応じて復帰する。
void avoidObstacle() {
  // delayしない段階制御にして、回避中もXBeeのSTOPを受信できるようにする。
  static int phase = 0;
  static unsigned long phaseStartTime = 0;
  static bool initialized = false;

  if (!initialized) {
    if (previousState == CARRY) {
      phase = 3;
    } else if (previousState == SEARCH) {
      phase = 10;
      if (searchStep >= 2000 && searchStep < 2360) {
        searchStep = (int)approachContactStartTime;
      }
      if (searchStep >= 100 && searchStep < 310) searchStep %= 10;
      if (sensors.floorColor == MY_TEAM_COLOR) {
        searchStep = 8;
      } else if (isOpponentColor(sensors.floorColor)) {
        searchStep = 9;
      } else if (sensors.floorColor == FLOOR_BLACK) {
        if (searchStep == 0) searchStep = 31;
        else if (searchStep == 2) searchStep = 41;
        else if (searchStep == 4) searchStep = 11;
        else if (searchStep == 6) searchStep = 21;
        else if (searchStep == 11 || searchStep == 12) searchStep = 33;
        else if (searchStep == 21 || searchStep == 22) searchStep = 43;
        else if (searchStep == 31 || searchStep == 32) searchStep = 13;
        else if (searchStep == 41 || searchStep == 42) searchStep = 23;
        else if (searchStep == 13) searchStep = 41;
        else if (searchStep == 23) searchStep = 31;
        else if (searchStep == 33) searchStep = 21;
        else if (searchStep == 43) searchStep = 11;
      }
    } else {
      phase = 0;
      searchStep = 10 + ((int)(readHeadingDeg() + 180.0f) % 360);
    }
    phaseStartTime = millis();
    initialized = true;
  }

  if (phase == 0 || phase == 3 || phase == 10) {
    stopMotors();
    if (millis() - phaseStartTime >= 100) {
      phase = phase == 0 ? 1 : (phase == 3 ? 4 : 11);
      phaseStartTime = millis();
    }
  } else if (phase == 1 || phase == 4 || phase == 11) {
    moveBackward(BACKWARD_SPEED);
    if (millis() - phaseStartTime >= 1000) {
      if (phase == 1) {
        phase = 2;
        phaseStartTime = millis();
      } else if (phase == 4) {
        phase = 5;
        phaseStartTime = millis();
      } else {
        stopMotors();
        sensors.targetDetected = false;
        targetPulseSamples = 0;
        searchStepStartTime = millis();
        searchInitialized = true;
        if (searchStep == 8 || searchStep == 9) {
          phase = 12;
        } else {
          state = SEARCH;
          initialized = false;
        }
      }
    }
  } else if (phase == 2) {
    if (turnToHeading((float)(searchStep - 10))) {
      phase = 6;
      phaseStartTime = millis();
    }
  } else if (phase == 6) {
    moveForward(FORWARD_SPEED);
    if (millis() - phaseStartTime >= 1000) {
      stopMotors();
      state = SEARCH;
      resetSearchPattern();
      initialized = false;
    }
  } else if (phase == 12) {
    if (turnToHeading(searchStep == 8 ? NORTH_HEADING_DEG
                                      : SOUTH_HEADING_DEG)) {
      searchStep = searchStep == 8 ? 0 : 2;
      searchStepStartTime = millis();
      state = SEARCH;
      initialized = false;
    }
  } else {
    stopMotors();
    state = CARRY;
    resetCarryMotion();
    initialized = false;
  }
}
