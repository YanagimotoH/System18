#include "RobotConfig.h"
#include <math.h>

static bool carryTurnFinished = false;
static unsigned long goalSequenceStartTime = 0;
static uint8_t goalSequencePhase = 0; // 0:後退開始、1:180度旋回

// 左右モーターの出力値を更新する。
void drive(int leftSpeed, int rightSpeed) {
  motorL = leftSpeed;
  motorR = rightSpeed;
}

// モーターを停止する。
void stopMotors() {
  drive(0, 0);
}

// 前進する。
void moveForward(int speed) {
  drive(speed, speed);
}

// 後退する。
void moveBackward(int speed) {
  drive(-speed, -speed);
}

// その場で旋回する。
void rotateInPlace(int speed) {
  drive(speed, -speed);
}

// カップ運搬開始時の状態を初期化する。
void resetCarryMotion() {
  carryTurnFinished = false;
}

// 自チーム色を3回連続で検出したらゴールと判定する。
bool detectGoal() {
  static uint8_t ownColorSamples = 0;

  if (sensors.floorColor == MY_TEAM_COLOR) {
    if (ownColorSamples < 3) ++ownColorSamples;
  } else {
    ownColorSamples = 0;
  }

  if (ownColorSamples >= 3) {
    ownColorSamples = 0;
    return true;
  }
  return false;
}

// カップ保持機構は未実装のため、現在は停止だけ行う。
void grabCup() {
  stopMotors();
}

// ゴール方位へ旋回し、向きが合ったらゴール検出まで直進する。
void carryToGoal() {
  if (detectGoal()) {
    stopMotors();
    sensors.targetDetected = false;
    goalSequencePhase = 0;
    resetCarryMotion();
    state = GOAL;
    return;
  }

  if (!carryTurnFinished) {
    if (!turnToHeading(CARRY_GOAL_HEADING_DEG, CARRY_HEADING_TOLERANCE_DEG)) return;
    carryTurnFinished = true;
    stopMotors();
    tone(buzzerPin, 880, 80);
    return;
  }

  moveForward(FORWARD_SPEED);
}

// 青色ゴール検出後、2秒後退して開始方向へ180度向き直し、探索を再開する。
void checkGoal() {
  if (goalSequencePhase == 0) {
    moveBackward(BACKWARD_SPEED);
    goalSequenceStartTime = millis();
    goalSequencePhase = 1;
    return;
  }

  if (goalSequencePhase == 1) {
    if (millis() - goalSequenceStartTime < 2000UL) {
      moveBackward(BACKWARD_SPEED);
      return;
    }
    stopMotors();
    goalSequencePhase = 2;
  }

  if (goalSequencePhase == 2 && !turnToHeading(SOUTH_HEADING_DEG)) {
    return;
  }

  stopMotors();
  sensors.goalDetected = true;
  sensors.targetDetected = false;
  goalSequencePhase = 0;
  resetCarryMotion();
  resetSearchPattern();
  state = SEARCH;
}

// 探索状態を初期化して再開する。
void resumeSearch() {
  stopMotors();
  sensors.targetDetected = false;
  resetCarryMotion();
  resetSearchPattern();
  state = SEARCH;
}
