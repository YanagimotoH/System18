#include <Arduino.h>
#include "RobotConfig.h"

// 起動時の向きを基準に45度ずつ確認し、最後に0度へ戻って一周する。
static const float SEARCH_HEADINGS_DEG[8] = {
  45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f, 0.0f
};
static uint8_t searchHeadingIndex = 0;
static bool searchForwarding = false;
static unsigned long searchPhaseStartTime = 0;
static bool searchInitialized = false;

static bool avoidInitialized = false;
static unsigned long avoidStartTime = 0;
static RobotState stateBeforeAvoid = SEARCH;

// 探索の一周・前進時間とカップ検出を初期化する。
void resetSearchPattern() {
  searchHeadingIndex = 0;
  searchForwarding = true;
  searchPhaseStartTime = millis();
  searchInitialized = true;
  avoidInitialized = false;
}

// 有効な距離が検出範囲内なら、1回の測定で対象候補とする。
bool detectTarget() {
  // ゴール直後は自陣色の領域を出るまでカップ候補を無視する。
  return !sensors.goalDetected &&
         sensors.distanceCm >= 0 &&
         sensors.distanceCm <= TARGET_DETECTION_RANGE_CM;
}

// 現在の超音波距離を返す。
int targetDistance() {
  return sensors.distanceCm;
}

// 方位を一周確認し、その後1秒前進する探索を繰り返す。
void search() {
  if (!searchInitialized) resetSearchPattern();

  if (sensors.targetDetected) {
    stopMotors();
    state = APPROACH;
    return;
  }

  if (searchForwarding) {
    if (millis() - searchPhaseStartTime < 1000UL) {
      moveForward(FORWARD_SPEED);
      return;
    }
    stopMotors();
    searchForwarding = false;
    searchHeadingIndex = 0;
    return;
  }

  if (turnToHeading(SEARCH_HEADINGS_DEG[searchHeadingIndex])) {
    ++searchHeadingIndex;
    if (searchHeadingIndex >= 8) {
      searchHeadingIndex = 0;
      searchForwarding = true;
      searchPhaseStartTime = millis();
    }
  }
}

// 検出した方向へ直進し、接近距離に入ったら運搬へ移る。
void approachTarget() {
  const int distanceCm = sensors.distanceCm;

  if (!sensors.targetDetected) {
    stopMotors();
    state = SEARCH;
    return;
  }

  if (distanceCm >= 0 && distanceCm <= APPROACH_LIMIT_CM) {
    stopMotors();
    sensors.targetDetected = false;
    resetCarryMotion();
    state = CARRY;
    return;
  }

  moveForward(100);
}

// Carry中に黒ラインへ入ったら、方位P制御で青陣地へ進む。
void avoidObstacle() {
  if (!avoidInitialized) {
    stateBeforeAvoid = previousState;
    avoidStartTime = millis();
    avoidInitialized = true;
  }

  if (stateBeforeAvoid == CARRY) {
    // 青陣地へ入ったらCarry側のゴール判定に戻す。
    if (detectGoal()) {
      stopMotors();
      sensors.goalDetected = true;
      sensors.targetDetected = false;
      state = GOAL;
      avoidInitialized = false;
      return;
    }

    // 黒ライン上は180度方向を保つよう、方位誤差をP補正して前進する。
    const float currentHeading = readHeadingDeg();
    if (currentHeading < 0.0f) {
      moveForward(FORWARD_SPEED);
      return;
    }

    float error = SOUTH_HEADING_DEG - currentHeading;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    const int correction = constrain(
        (int)(error * getHeadingRotationSign()), -80, 80);
    drive(FORWARD_SPEED + correction, FORWARD_SPEED - correction);
    return;
  }

  if (millis() - avoidStartTime < 500UL) {
    moveBackward(BACKWARD_SPEED);
    return;
  }

  stopMotors();
  state = stateBeforeAvoid;
  avoidInitialized = false;
}
