#include <Arduino.h>
#include "RobotConfig.h"

static int searchStep = 0; // 探索パターンを進めるためのステップ番号
static unsigned long searchStepStartTime = 0; // 探索パターンのステップ開始時刻を記録する変数
static bool searchInitialized = false; // 探索パターンの初期化が完了したかどうかを示すフラグ

static bool classifyingObject = false; // 動体か静止物体かを判定する段階
static bool confirmingTarget = false;  // 候補が視界に残っていることを確認する段階
static unsigned long targetConfirmStartTime = 0; // 候補が視界に残っていることを確認する段階の開始時刻を記録する変数
static bool settlingTarget = false; // 停止後、距離が安定するまで待つ段階
static unsigned long targetSettleStartTime = 0; // 停止後、距離が安定するまで待つ段階の開始時刻を記録する変数
static unsigned long classificationStartTime = 0; // 動体か静止物体かを判定する段階の開始時刻を記録する変数
static unsigned long lastClassificationSampleTime = 0; // 動体か静止物体かを判定する段階で、距離を確認した最後の時刻を記録する変数
static int minimumObservedDistance = 0; // 動体か静止物体かを判定する段階で、観測された最小距離を記録する変数
static int maximumObservedDistance = 0; // 動体か静止物体かを判定する段階で、観測された最大距離を記録する変数
static bool approachConfirming = false; // 0～3cmを維持できているか確認中
static unsigned long approachContactStartTime = 0; // 0～3cmの確認開始時刻
static uint8_t approachLostSampleCount = 0; // 接近中の範囲外連続回数

// サーチパターンの初期化を行う関数
void resetSearchPattern() {
  searchStep = 0;
  searchStepStartTime = millis();
  searchInitialized = true;
  classifyingObject = false;
  confirmingTarget = false;
  settlingTarget = false;
  approachConfirming = false;
  approachLostSampleCount = 0;
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
  const unsigned long ROTATE_TIME = SWEEP_360_TIME_MS;

  // 探索へ戻った直後も0～1cmに対象がある場合は、カップを保持中とみなし
  // SEARCHへ進まず運搬状態を維持する。
  if (sensors.distanceCm >= 0 && sensors.distanceCm <= 1) {
    stopMotors();
    sensors.targetDetected = false;
    approachConfirming = false;
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
  // 候補が視界に残っていることを確認する。
  // 旋回中に候補が視界から外れた場合は、再度確認をやり直す。
  if (classifyingObject) {
    stopMotors();

    if (sensors.distanceCm <= 0 ||
        sensors.distanceCm > TARGET_DETECTION_RANGE_CM) {
      classifyingObject = false;
      searchInitialized = false;
      sensors.targetDetected = false;
      confirmingTarget = false;
      settlingTarget = false;
      // 測距不能・検出範囲外は動体の根拠にせず、探索をやり直す。
      state = SEARCH;
      return;
    }
    // 動体か静止物体かを判定する段階で、距離の最大値と最小値を記録する。
    if (millis() - lastClassificationSampleTime >=
        OBJECT_SAMPLE_INTERVAL_MS) {
      lastClassificationSampleTime = millis();
      if (sensors.distanceCm < minimumObservedDistance)
        minimumObservedDistance = sensors.distanceCm;
      if (sensors.distanceCm > maximumObservedDistance)
        maximumObservedDistance = sensors.distanceCm;
    }
    // 動体か静止物体かを判定する段階で、一定時間経過したら距離変化を確認する。
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
  // 探索パターンの初期化を行う。
  if (!searchInitialized) {
    searchStep = 0;
    searchStepStartTime = millis();
    searchInitialized = true;
  }
  // 候補が視界に残っていることを確認する段階。
  // 一定時間経過したら停止して距離が安定するのを待つ。
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

  // 少し前進した後、その場で約360度回転する探索パターンを繰り返す。
  unsigned long duration = (searchStep == 0) ? FORWARD_TIME : ROTATE_TIME;

  if (millis() - searchStepStartTime >= duration) {
    searchStep = (searchStep + 1) % 2;
    searchStepStartTime = millis();
  }

  if (searchStep == 0) {
    moveForward(FORWARD_SPEED);
  } else {
    // その場で約360度回転する。
    rotateInPlace(ROTATE_SPEED);
  }
}

void approachTarget() {
  int currentDistanceCm = sensors.distanceCm;

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

  // 範囲外の値を1～2回受けても接近を続け、3回連続した場合だけ探索へ戻る。
  if (currentDistanceCm < 0 ||
      currentDistanceCm > TARGET_DETECTION_RANGE_CM) {
    approachLostSampleCount++;

    if (approachLostSampleCount >= APPROACH_LOST_CONFIRM_COUNT) {
      approachLostSampleCount = 0;
      sensors.targetDetected = false;
      stopMotors();
      resetSearchPattern();
      state = SEARCH;
      return;
    }

    // 距離を読めない場合は停止し、大きな距離値の場合は接近を続ける。
    if (currentDistanceCm < 0) {
      stopMotors();
    } else {
      moveForward(FORWARD_SPEED);
    }
    return;
  }

  approachLostSampleCount = 0;
  moveForward(FORWARD_SPEED);
}

// 障害物回避の段階制御を行う関数
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
