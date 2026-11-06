
#include "RobotConfig.h"
#include <math.h>

// 山登りの段階
enum ClimbPhase {
  CLIMB_APPROACH, // 山の斜面を探す
  CLIMB_UP,       // 山を登る
  CLIMB_TOP,      // 山頂で停止する
  CLIMB_DOWN,     // 山を下る
  CLIMB_FINISH    // 山登りを終了する
};

static ClimbPhase climbPhase = CLIMB_APPROACH;

// 判定時間を記録する
static unsigned long detectStartTime = 0;
static unsigned long topWaitStartTime = 0;

// 下り斜面に入ったかを記録する
static bool slopeDetected = false;

// 山登り中のモーター速度
const int CLIMB_FORWARD_SPEED = 150;

// 平地で測定した加速度の基準値
const int CLIMB_FLAT_X = -50;
const int CLIMB_FLAT_Y = 0;

// 前後方向の傾きを判定する閾値
const int CLIMB_UP_THRESHOLD = 1200;
const int CLIMB_DOWN_THRESHOLD = -1200;

// 平地を判定する閾値
const int CLIMB_FLAT_THRESHOLD = 700;

// 判定を安定させる時間
const unsigned long CLIMB_DETECT_TIME = 100;

// 山頂で停止する時間
const unsigned long CLIMB_TOP_WAIT_TIME = 1000;

// 山登りの状態を初期化する
void resetClimbMission() {
  climbPhase = CLIMB_APPROACH;
  detectStartTime = 0;
  topWaitStartTime = 0;
  slopeDetected = false;
}

// 山登りミッションを1周期進める
void updateClimbMission() {

  // ミッションが停止している場合はモーターを止める
  if (!robotEnabled || !hillMissionActive) {
    stopMotors();
    return;
  }

  // 加速度センサーの値を取得する
  int accelX = readAccelX();
  int accelY = readAccelY();

  // 平地を基準にした前後方向の傾き
  int pitchValue = accelX - CLIMB_FLAT_X;

  switch (climbPhase) {

    case CLIMB_APPROACH:

      // 山の斜面を探すために前進する
      moveForward(CLIMB_FORWARD_SPEED);

      // 前後方向の傾きが続いたら斜面に入ったと判断する
      if (pitchValue >= CLIMB_UP_THRESHOLD) {

        if (detectStartTime == 0) {
          detectStartTime = millis();
        }

        // 一定時間斜面を検出したら登山を開始する
        if (millis() - detectStartTime >= CLIMB_DETECT_TIME) {
          detectStartTime = 0;
          climbPhase = CLIMB_UP;

          Serial1.println("ID=1,STATUS=HILL_STARTED");
        }

      } else {
        detectStartTime = 0;
      }

      break;

    case CLIMB_UP:

      // 山頂に到達するまで前進する
      moveForward(CLIMB_FORWARD_SPEED);

      // 傾きが小さくなったら山頂に近づいたと判断する
      if (abs(pitchValue) <= CLIMB_FLAT_THRESHOLD) {

        if (detectStartTime == 0) {
          detectStartTime = millis();
        }

        // 一定時間平地を検出したら山頂で停止する
        if (millis() - detectStartTime >= CLIMB_DETECT_TIME) {
          stopMotors();
          detectStartTime = 0;
          topWaitStartTime = millis();
          climbPhase = CLIMB_TOP;
        }

      } else {
        detectStartTime = 0;
      }

      break;

    case CLIMB_TOP:

      // 山頂で停止する
      stopMotors();

      // 一定時間停止したら下山を開始する
      if (millis() - topWaitStartTime >= CLIMB_TOP_WAIT_TIME) {
        slopeDetected = false;
        climbPhase = CLIMB_DOWN;
      }

      break;

    case CLIMB_DOWN:

      // 登ってきた方向のまま前進して下山する
      moveForward(CLIMB_FORWARD_SPEED);

      // 下り斜面に入ったことを検出する
      if (pitchValue <= CLIMB_DOWN_THRESHOLD) {
        slopeDetected = true;
      }

      // 下り斜面を抜けて平地に戻ったか判定する
      if (slopeDetected && abs(pitchValue) <= CLIMB_FLAT_THRESHOLD) {

        if (detectStartTime == 0) {
          detectStartTime = millis();
        }

        // 一定時間平地を検出したら下山を終了する
        if (millis() - detectStartTime >= CLIMB_DETECT_TIME) {
          stopMotors();
          detectStartTime = 0;
          climbPhase = CLIMB_FINISH;
        }

      } else {
        detectStartTime = 0;
      }

      break;

    case CLIMB_FINISH:

      // モーターを停止する
      stopMotors();

      // 山登り終了をProcessingへ通知する
      hillMissionActive = false;
      Serial1.println("ID=1,STATUS=HILL_DONE");

      // 通常の探索に戻る
      resetSearchPattern();
      state = SEARCH;
      resetClimbMission();

      break;
  }
}
