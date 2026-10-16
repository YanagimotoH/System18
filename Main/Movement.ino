#include "RobotConfig.h"

void drive(int leftSpeed, int rightSpeed) {
    motorL = leftSpeed;
    motorR = rightSpeed;
}

void stopMotors() {
    drive(0, 0);
}

void moveForward(int speed) {
    drive(speed, speed);
}

void moveBackward(int speed) {
    drive(-speed, -speed);
}

void rotateInPlace(int speed) {
    drive(speed, -speed);
}

static bool carryTurnStarted = false;
static bool carryTurnFinished = false;
static unsigned long carryTurnStartTime = 0;
static unsigned long carryForwardStartTime = 0;

void resetCarryMotion() {
    carryTurnStarted = false;
    carryTurnFinished = false;
    carryTurnStartTime = 0;
    carryForwardStartTime = 0;
}

// ゴール・保持機能を実装するまでの仮関数。
// 現時点では誤走行を防ぐため、関連状態に入った場合は停止する。
bool detectGoal() {
  // TODO: ゴール判定条件が決まったら実装する。
  return false;
}

void grabCup() {
  // TODO: カップ保持機構が決まったら実装する。
  stopMotors();
}

void carryToGoal() {
  if (carryTurnFinished) {
    moveForward(FORWARD_SPEED);

    // 回転後に一定時間進み、前方に物体がいなくなったら探索へ戻る。
    bool objectHasCleared =
        sensors.distanceCm < 0 ||
        sensors.distanceCm >= CARRY_CLEAR_DISTANCE_CM;
    bool minimumForwardTimePassed =
        millis() - carryForwardStartTime >= CARRY_MIN_FORWARD_TIME_MS;

    if (minimumForwardTimePassed && objectHasCleared) {
      stopMotors();
      sensors.targetDetected = false;
      state = SEARCH;
      resetSearchPattern();
      resetCarryMotion();
    }
    return;
  }

  if (!carryTurnStarted) {
    carryTurnStarted = true;
    carryTurnStartTime = millis();
  }

  if (millis() - carryTurnStartTime < ROTATE_180_TIME_MS) {
    rotateInPlace(ROTATE_SPEED);
  } else {
    carryTurnFinished = true;
    carryForwardStartTime = millis();
    moveForward(FORWARD_SPEED);
  }
}

void checkGoal() {
  // TODO: ゴール成立条件と得点処理を実装する。
  stopMotors();
}

void resumeSearch() {
  // TODO: ゴール後の再探索動作を実装する。
  stopMotors();
  state = SEARCH;
  resetSearchPattern();
}
