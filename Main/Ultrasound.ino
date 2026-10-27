#include "RobotConfig.h"

// 超音波センサから距離を取得する関数
int distance() {
  unsigned long interval;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  interval = pulseIn(echo, HIGH, 23071);
  
  // 超音波センサから距離を取得する。
  if (interval == 0) {
    // 距離を取得できなかった場合、-1を返す。
    return -1;
  }
  // 距離を計算する。
  int distanceCm =
    (0.61 * 25 + 331.5) * interval / 10000 / 2;
  // 超音波センサの近距離側の測定範囲を接触距離0cmとして扱う。
  if (distanceCm <= 2) {
    return 0;
  }
  return distanceCm;
}

// 障害物があるかどうかを判定する関数
bool isObstacle(int distanceCm) {
  // OBSTACLE_LIMIT_CM以内の距離を検出した場合にtrueを返す。
  return distanceCm > 0 &&
         distanceCm < OBSTACLE_LIMIT_CM;
}

// 距離計測結果から障害物の有無を判定する
bool detection() {
  // TARGET_DETECTION_RANGE_CM以内の距離を検出した場合にtrueを返す。
  return sensors.distanceCm > 0 &&
         sensors.distanceCm <= TARGET_DETECTION_RANGE_CM;
}

// 探索中の基本走行。障害物回避状態への切替はExploration.inoが行う
void transport() {
  if (detection()) {
    stopMotors();
    state = SEARCH;
  } else {
    moveForward(FORWARD_SPEED);
  }
}
