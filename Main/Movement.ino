#include "RobotConfig.h"
#include <math.h>

static bool carryTurnFinished = false;
static unsigned long goalSequenceStartTime = 0;
static uint8_t goalSequencePhase = 0; // 0:後退開始、1:180度旋回
static float goalReturnHeadingDeg = -1.0f;

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
  goalReturnHeadingDeg = -1.0f;
  resetCupLost();
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
    goalReturnHeadingDeg = readHeadingDeg();
    return true;
  }
  return false;
}

// カップ保持機構は未実装のため、現在は停止だけ行う。
void grabCup() {
  stopMotors();
}

// 走行開始時の向きを0°として、CARRY_GOAL_HEADING_DEGへ正確に旋回し、ゴール検出まで直進する。
// カップが前方から無くなったら探索へ戻る。
void carryToGoal() {
  if (cupLost()) {
    resumeSearch();
    return;
  }

  if (detectGoal()) {
    stopMotors();
    sensors.targetDetected = false;
    goalSequencePhase = 0;
    goalSequenceStartTime = millis();
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

  // 前進中にゴール方位から大きくずれたら、止まってもう一度向き直す。
  const float heading = readHeadingDeg();
  if (heading >= 0.0f) {
    float error = CARRY_GOAL_HEADING_DEG - heading;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    if (fabs(error) > CARRY_REALIGN_DEG) {
      carryTurnFinished = false;
      stopMotors();
      return;
    }
  }

  moveForward(FORWARD_SPEED);
}

// 自チーム色検出後、2秒後退してゴール進入時と反対を向き、探索を再開する。
void checkGoal() {
  if (goalSequencePhase == 0) {
    // 停止してカップが前方にあるか確認する。無ければ探索へ戻る。
    stopMotors();
    if (cupLost()) {
      resumeSearch();
      return;
    }
    if (millis() - goalSequenceStartTime < GOAL_CONFIRM_MS) return;
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

  if (goalSequencePhase == 2) {
    if (goalReturnHeadingDeg < 0.0f) {
      stopMotors();
      return;
    }
    float oppositeHeadingDeg = goalReturnHeadingDeg + 180.0f;
    if (oppositeHeadingDeg >= 360.0f) oppositeHeadingDeg -= 360.0f;
    if (!turnToHeading(oppositeHeadingDeg)) return;
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
  goalSequencePhase = 0;
  sensors.targetDetected = false;
  resetCarryMotion();
  resetSearchPattern();
  state = SEARCH;
}