#include "RobotConfig.h"
#include <math.h>

// 山登りミッションの各段階
enum ClimbPhase {
  CLIMB_APPROACH, // 山の斜面を探す
  CLIMB_UP,       // 斜面を登る
  CLIMB_SEARCH,   // 頂上で360度回転してカップを探す
  CLIMB_WAIT,     // カップ回収の仮処理
  CLIMB_TURN_DOWN,// 登ってきた方向と反対を向く
  CLIMB_DOWN,     // 斜面を下る
  CLIMB_FINISH    // 山登り終了
};

static ClimbPhase climbPhase = CLIMB_APPROACH;

static float climbStartHeading = 0.0f;
static float previousSearchHeading = -1.0f;
static float searchedAngle = 0.0f;

static unsigned long phaseStartTime = 0;
static unsigned long flatDetectStartTime = 0;

static bool slopeDetected = false;

// 加速度センサーの値から斜面を判定するための閾値。
// 平地は約-115、前傾15度では約2931だった測定値を参考にしている。
const int CLIMB_UP_THRESHOLD = 1200;
const int CLIMB_DOWN_THRESHOLD = -1200;
const int CLIMB_FLAT_THRESHOLD = 900;

// 斜面・平地の判定を安定させるための時間
const unsigned long CLIMB_DETECT_TIME = 400;

// 山の頂上でカップを探すときの旋回速度
const int CLIMB_SEARCH_SPEED = 70;

// カップ回収の仮処理時間
const unsigned long CUP_PLACEHOLDER_TIME = 1500;

// 角度の差を-180～180度に収める。
static float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// 山登りミッションの初期状態にする。
void resetClimbMission() {
  climbPhase = CLIMB_APPROACH;
  climbStartHeading = 0.0f;
  previousSearchHeading = -1.0f;
  searchedAngle = 0.0f;
  phaseStartTime = 0;
  flatDetectStartTime = 0;
  slopeDetected = false;
}

// 頂上で360度回転しながら周囲を確認する。
static bool searchAtHillTop() {
  float currentHeading = readHeadingDeg();

  if (currentHeading < 0.0f) {
    stopMotors();
    return false;
  }

  // 最初の方位を記録する。
  if (previousSearchHeading < 0.0f) {
    previousSearchHeading = currentHeading;
  }

  // 前回からの角度変化を加算する。
  float difference = normalizeAngle(currentHeading - previousSearchHeading);
  searchedAngle += fabs(difference);
  previousSearchHeading = currentHeading;

  // 360度近く回転したら探索を終了する。
  if (searchedAngle >= 350.0f) {
    stopMotors();
    return true;
  }

  rotateInPlace(CLIMB_SEARCH_SPEED);
  return false;
}

// 山登りミッションを1周期進める。
// メインループから繰り返し呼び出す。
void updateClimbMission() {
  if (!robotEnabled || !hillMissionActive) {
    stopMotors();
    return;
  }

  int accelX = readAccelX();

  switch (climbPhase) {

    case CLIMB_APPROACH:
      // 山の斜面に到達するまでは前進する。
      moveForward(FORWARD_SPEED);

      // 前傾が一定以上になったら斜面に入ったと判断する。
      if (accelX >= CLIMB_UP_THRESHOLD) {
        if (flatDetectStartTime == 0) {
          flatDetectStartTime = millis();
        }

        if (millis() - flatDetectStartTime >= CLIMB_DETECT_TIME) {
          // 斜面に入った時点の方位を記録する。
          climbStartHeading = readHeadingDeg();

          if (climbStartHeading >= 0.0f) {
            slopeDetected = true;
            climbPhase = CLIMB_UP;
            flatDetectStartTime = 0;
            Serial1.println("ID=1,STATUS=HILL_STARTED");
          } else {
            stopMotors();
          }
        }
      } else {
        flatDetectStartTime = 0;
      }
      break;

    case CLIMB_UP:
      // 斜面を登り続ける。
      moveForward(FORWARD_SPEED);

      // 斜面を登った後、平地に戻ったら頂上と判断する。
      if (abs(accelX) <= CLIMB_FLAT_THRESHOLD) {
        if (flatDetectStartTime == 0) {
          flatDetectStartTime = millis();
        }

        if (millis() - flatDetectStartTime >= CLIMB_DETECT_TIME) {
          stopMotors();
          climbPhase = CLIMB_SEARCH;
          previousSearchHeading = -1.0f;
          searchedAngle = 0.0f;
          flatDetectStartTime = 0;
        }
      } else {
        flatDetectStartTime = 0;
      }
      break;

    case CLIMB_SEARCH:
      // 頂上で360度回転してカップを探す。
      if (searchAtHillTop()) {
        stopMotors();
        phaseStartTime = millis();
        climbPhase = CLIMB_WAIT;
      }
      break;

    case CLIMB_WAIT:
      // 現在は他メンバーのカップ回収処理が未完成なので仮に待機する。
      stopMotors();

      if (millis() - phaseStartTime >= CUP_PLACEHOLDER_TIME) {
        climbPhase = CLIMB_TURN_DOWN;
      }
      break;

    case CLIMB_TURN_DOWN: {
      // 登ってきた方向と反対の方位を計算する。
      float downHeading = climbStartHeading + 180.0f;

      if (downHeading >= 360.0f) {
        downHeading -= 360.0f;
      }

      // 反対方向を向くまで旋回する。
      if (turnToHeading(downHeading)) {
        flatDetectStartTime = 0;
        climbPhase = CLIMB_DOWN;
      }
      break;
    }

    case CLIMB_DOWN:
      // 登ってきた方向へ戻るように前進する。
      moveForward(FORWARD_SPEED);

      // 下り斜面の傾きを検出する。
      if (accelX <= CLIMB_DOWN_THRESHOLD) {
        slopeDetected = true;
      }

      // 下り斜面を抜けて平地に戻ったら終了する。
      if (slopeDetected && abs(accelX) <= CLIMB_FLAT_THRESHOLD) {
        if (flatDetectStartTime == 0) {
          flatDetectStartTime = millis();
        }

        if (millis() - flatDetectStartTime >= CLIMB_DETECT_TIME) {
          stopMotors();
          climbPhase = CLIMB_FINISH;
        }
      } else {
        flatDetectStartTime = 0;
      }
      break;

    case CLIMB_FINISH:
      stopMotors();

      // 山登り終了をProcessingへ通知する。
      hillMissionActive = false;
      Serial1.println("ID=1,STATUS=HILL_DONE");

      // 通常の探索に戻る。
      resetSearchPattern();
      state = SEARCH;
      resetClimbMission();
      break;
  }
}