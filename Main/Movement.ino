#include "RobotConfig.h"

// モーターの速度を設定する関数
void drive(int leftSpeed, int rightSpeed) {
    motorL = leftSpeed;
    motorR = rightSpeed;
}
// モーターを停止する関数
void stopMotors() {
    drive(0, 0);
}
// 前進する関数
void moveForward(int speed) {
    drive(speed, speed);
}
// 後退する関数
void moveBackward(int speed) {
    drive(-speed, -speed);
}
// その場で回転する関数
void rotateInPlace(int speed) {
    drive(speed, -speed);
}

static bool carryTurnStarted = false;  // カップ到達後の回転開始
static bool carryTurnFinished = false; // カップ到達後の回転完了
static unsigned long carryTurnStartTime = 0;    // カップ到達後、回転開始時刻
static unsigned long carryForwardStartTime = 0; // カップ到達後、回転完了時刻

// カップ到達後の動作をリセットする関数
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

// カップ到達後の動作を行う関数
void carryToGoal() {
  // 距離の-1（未検出）ではSEARCHへ戻さず、ゴール成立までCARRYを維持する。
  if (detectGoal()) {
    stopMotors();
    state = GOAL;
    resetCarryMotion();
    return;
  }

  // ゴールが未成立の間は、回転後もCARRY状態で前進を続ける。
  if (carryTurnFinished) {
    moveForward(FORWARD_SPEED);
    bool objectHasCleared =
        sensors.distanceCm >= CARRY_CLEAR_DISTANCE_CM;
    bool minimumForwardTimePassed =
        millis() - carryForwardStartTime >= CARRY_MIN_FORWARD_TIME_MS;

    // 前進最低時間を過ぎ、測定距離が40cm以上になったら探索へ戻る。
    if (minimumForwardTimePassed && objectHasCleared) {
      stopMotors();
      sensors.targetDetected = false;
      state = SEARCH;
      resetSearchPattern();
      resetCarryMotion();
    }
    return;
  }
  // 回転が完了していない場合は、回転を行う。
  if (!carryTurnStarted) {
    carryTurnStarted = true;       // 回転開始
    carryTurnStartTime = millis(); // 回転開始時刻を記録
  }
  if (millis() - carryTurnStartTime < ROTATE_180_TIME_MS) {
    rotateInPlace(ROTATE_SPEED);      // 回転時間内
  } else {
    carryTurnFinished = true;         // 回転完了
    carryForwardStartTime = millis(); // 前進開始時刻を記録
    moveForward(FORWARD_SPEED);       // 回転後に前進
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
