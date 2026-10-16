#include <Arduino.h>
#include "RobotConfig.h"

static int searchStep = 0;
static unsigned long searchStepStartTime = 0;
static bool searchInitialized = false;
static bool classifyingObject = false;
static bool confirmingTarget = false;
static unsigned long targetConfirmStartTime = 0;
static bool settlingTarget = false;
static unsigned long targetSettleStartTime = 0;
static unsigned long classificationStartTime = 0;
static unsigned long lastClassificationSampleTime = 0;
static int minimumObservedDistance = 0;
static int maximumObservedDistance = 0;
static bool confirmingApproachContact = false;
static unsigned long approachContactStartTime = 0;

void resetSearchPattern() {
  searchStep = 0;
  searchStepStartTime = millis();
  searchInitialized = true;
  classifyingObject = false;
  confirmingTarget = false;
  settlingTarget = false;
  confirmingApproachContact = false;
}

bool detectTarget() {
  // 検出範囲内の物体を候補として扱い、静止/移動の判定はsearch()で行う。
  return detection();
}

int targetDistance() {
  // ターゲット検出条件が決まった後、超音波距離を対象距離として使う。
  return sensors.distanceCm;
}

void search() {
  const unsigned long FORWARD_TIME = 500;
  const unsigned long ROTATE_TIME = SWEEP_150_TIME_MS;

  // 探索へ戻った直後も0～1cmに対象がある場合は、カップを保持中とみなし
  // SEARCHへ進まず運搬状態を維持する。
  if (sensors.distanceCm >= 0 && sensors.distanceCm <= 1) {
    stopMotors();
    sensors.targetDetected = false;
    confirmingApproachContact = false;
    resetCarryMotion();
    state = CARRY;
    return;
  }

  // 候補を旋回中に一定時間見続けてから停止し、測距が落ち着くのを待つ。
  // 旋回中の距離変化を、対象自身の移動と誤認しないための段階。
  if (settlingTarget) {
    stopMotors();
    if (millis() - targetSettleStartTime >= TARGET_SETTLE_TIME_MS) {
      settlingTarget = false;
      classifyingObject = true;
      classificationStartTime = millis();
      lastClassificationSampleTime = millis();
      minimumObservedDistance = sensors.distanceCm;
      maximumObservedDistance = sensors.distanceCm;
    }
    return;
  }

  if (classifyingObject) {
    stopMotors();

    if (sensors.distanceCm <= 0 ||
        sensors.distanceCm > TARGET_DETECTION_RANGE_CM) {
      classifyingObject = false;
      searchInitialized = false;
      sensors.targetDetected = false;
      state = AVOID;
      return;
    }

    if (millis() - lastClassificationSampleTime >=
        OBJECT_SAMPLE_INTERVAL_MS) {
      lastClassificationSampleTime = millis();
      if (sensors.distanceCm < minimumObservedDistance)
        minimumObservedDistance = sensors.distanceCm;
      if (sensors.distanceCm > maximumObservedDistance)
        maximumObservedDistance = sensors.distanceCm;
    }

    if (millis() - classificationStartTime >= OBJECT_MOTION_WINDOW_MS) {
      int distanceChange = maximumObservedDistance - minimumObservedDistance;
      classifyingObject = false;
      sensors.targetDetected = false;

      if (distanceChange >= OBJECT_MOTION_THRESHOLD_CM) {
        // 動く物体をロボット候補として回避する。
        state = AVOID;
      } else {
        // 距離が安定している物体をカップ候補として接近する。
        sensors.targetDetected = true;
        state = APPROACH;
      }
      return;
    }
    return;
  }

  if (!searchInitialized) {
    searchStep = 0;
    searchStepStartTime = millis();
    searchInitialized = true;
  }

  if (sensors.targetDetected) {
    if (!confirmingTarget) {
      confirmingTarget = true;
      targetConfirmStartTime = millis();
    } else if (millis() - targetConfirmStartTime >= TARGET_CONFIRM_TIME_MS) {
      confirmingTarget = false;
      stopMotors();
      settlingTarget = true;
      targetSettleStartTime = millis();
      return;
    }
  } else {
    // 視界から外れた候補は連続確認をやり直す。
    confirmingTarget = false;
  }

  // 前進後、左へ約150度振って正面へ戻し、右へ約150度振って戻す。
  // 検出されなければこの探索パターンを繰り返す。
  unsigned long duration = (searchStep == 0) ? FORWARD_TIME : ROTATE_TIME;

  if (millis() - searchStepStartTime >= duration) {
    searchStep = (searchStep + 1) % 5;
    searchStepStartTime = millis();
  }

  if (searchStep == 0) {
    moveForward(FORWARD_SPEED);
  } else if (searchStep == 1 || searchStep == 4) {
    // 左旋回: 左へ振る、または右側から正面へ戻る
    rotateInPlace(ROTATE_SPEED);
  } else {
    // 右旋回: 左側から正面へ戻る、または右へ振る
    drive(-ROTATE_SPEED, ROTATE_SPEED);
  }
}

void approachTarget() {
  if (!sensors.targetDetected) {
    confirmingApproachContact = false;
    state = SEARCH;
    return;
  }

  // 対象が50cm以上離れたら接近を中断し、探索をやり直す。
  if (sensors.targetDistanceCm >= TARGET_DETECTION_RANGE_CM) {
    confirmingApproachContact = false;
    sensors.targetDetected = false;
    stopMotors();
    resetSearchPattern();
    state = SEARCH;
    return;
  }

  // 接触距離に達したら停止して1秒待ち、待機終了時の距離で判定する。
  if (confirmingApproachContact) {
    stopMotors();

    if (millis() - approachContactStartTime < 1000) {
      return;
    }

    confirmingApproachContact = false;
    if (sensors.targetDistanceCm == 0 || sensors.targetDistanceCm == 1) {
      sensors.targetDetected = false;
      resetCarryMotion();
      state = CARRY;
    } else if (sensors.targetDistanceCm > 1) {
      // 1秒後に離れていたら、再び接近して確認し直す。
      moveForward(FORWARD_SPEED);
    }
    return;
  }

  if (sensors.targetDistanceCm == 0 || sensors.targetDistanceCm == 1) {
    confirmingApproachContact = true;
    approachContactStartTime = millis();
    stopMotors();
    return;
  }

  // 距離が読めない間は前進せず、読み取れたら接近を続ける。
  if (sensors.targetDistanceCm < 0) {
    stopMotors();
    return;
  }

  moveForward(FORWARD_SPEED);
}


void avoidObstacle() {
  // delayしない段階制御にして、回避中もXBeeのSTOPを受信できるようにする。
  static int phase = 0;
  static unsigned long phaseStartTime = 0;
  static bool initialized = false;

  if (!initialized) {
    phase = 0;
    phaseStartTime = millis();
    initialized = true;
  }

  if (phase == 0) {
    stopMotors();
    if (millis() - phaseStartTime >= 100) {
      phase = 1;
      phaseStartTime = millis();
    }
  } else if (phase == 1) {
    moveBackward(BACKWARD_SPEED);
    if (millis() - phaseStartTime >= 300) {
      phase = 2;
      phaseStartTime = millis();
    }
  } else {
    rotateInPlace(ROTATE_SPEED);
    if (millis() - phaseStartTime >= 500) {
      stopMotors();
      state = SEARCH;
      searchInitialized = false;
      initialized = false;
    }
  }
}
