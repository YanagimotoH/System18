#include "RobotConfig.h"
#include <math.h>

static bool carryTurnFinished = false; // カップ到達後の回転完了
static unsigned long carryForwardStartTime = 0; // カップ到達後、回転完了時刻
static unsigned long goalSequenceStartTime = 0; // ゴール後の各段階を開始した時刻
static int goalSequencePhase = 0; // ゴール後処理の段階（停止、後退、向き直し）

// 左右モーターの出力値を更新する。
void drive(int leftSpeed, int rightSpeed) {
    motorL = leftSpeed;
    motorR = rightSpeed;
}
// 左右モーターの出力を0にする。
void stopMotors() {
    drive(0, 0);
}
// 左右を同じ指定速度にして前進する。
void moveForward(int speed) {
    drive(speed, speed);
}
// 左右を同じ負の速度にして後退する。
void moveBackward(int speed) {
    drive(-speed, -speed);
}
// 左右を逆方向に回してその場旋回する。
void rotateInPlace(int speed) {
    drive(speed, -speed);
}

// 次の運搬開始に備えて旋回・前進の進行状態を初期化する。
void resetCarryMotion() {
    carryTurnFinished = false;
    carryForwardStartTime = 0;
}

// 自チーム色を3回連続で検出してゴール到達を判定する暫定処理。
bool detectGoal() {
  // 自陣色を連続3回確認して誤判定を抑える。
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

// 仮処理。保持機構が未実装のため、現在はロボットを停止するだけ。
void grabCup() {
  // TODO: 他メンバーによるカップ保持機構の仕様確定後に実装する。
  stopMotors();
}

// 暫定運搬処理。現在は南向きへ旋回して前進するだけで、山登り・カップ回収は未実装。
void carryToGoal() {
  // ゴール検出は距離センサの状態と独立して行う。
  if (detectGoal()) {
    stopMotors();
    state = GOAL;
    resetCarryMotion();
    return;
  }

  // CARRYへ入った最初の周期は必ず停止して、次の周期から方位合わせを始める。
  if (!carryTurnFinished && carryForwardStartTime == 0) {
    stopMotors();
    carryForwardStartTime = millis();
    return;
  }

  // 開始時の向きから180度（青ゴール側）へ向き切るまで、その場で旋回する。
  // 方位が合う前に前進へ移らない。
  if (!carryTurnFinished && turnToHeading(SOUTH_HEADING_DEG)) {
    carryTurnFinished = true;
    carryForwardStartTime = millis();
    tone(buzzerPin, 880, 80); // 南向き到達を短く通知
  }
  if (!carryTurnFinished) return;

  // 方位が合ってから青ゴールまで運搬する。
  moveForward(FORWARD_SPEED);
  // 超音波の反射が途切れても走行を続け、最大30秒で探索へ戻る。
  if (millis() - carryForwardStartTime >= 30000UL) {
    stopMotors();
    resetCarryMotion();
  }
}

// ゴール到達後に後退し、内向きに向き直して探索を再開する。
void checkGoal() {
  // 自陣ゴールを検出したら1.5秒後退し、ゴール処理中に内向きへ180度回転する。
  if (goalSequencePhase == 0) {
    stopMotors();
    goalSequenceStartTime = millis();
    goalSequencePhase = 1;
  } else if (goalSequencePhase == 1) {
    moveBackward(BACKWARD_SPEED);
    if (millis() - goalSequenceStartTime >= 1500) {
      goalSequencePhase = 2;
    }
  } else if (goalSequencePhase == 2) {
    if (turnToHeading(NORTH_HEADING_DEG)) {
      goalSequencePhase = 0;
      sensors.goalDetected = true;
      sensors.targetDetected = false;
      resetCarryMotion();
      resetSearchPattern();
      state = SEARCH;
    }
  }
}

// 探索状態を初期化して再開する暫定処理。
void resumeSearch() {
  // TODO: ゴール後の専用再探索動作が必要になったら追加する。
  stopMotors();
  state = SEARCH;
  resetSearchPattern();
}
