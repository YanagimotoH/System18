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

// 方向補正の強さ
const int CLIMB_STEER_DIVISOR = 40;
// 方向補正の最大値
const int CLIMB_MAX_STEER = 30;

// 平地で測定した加速度の基準値
const int CLIMB_FLAT_X = -50;
const int CLIMB_FLAT_Y = 0;

// 前後方向の傾きを判定する閾値
const int CLIMB_UP_THRESHOLD = 1200;
const int CLIMB_DOWN_THRESHOLD = -1200;
// 前回の前後方向の傾きを記録する
static int previousPitchValue = 0;
// 前後方向の傾きが急激に変化したか判定するための設定
const int CLIMB_TOP_CHANGE_THRESHOLD = 300; // 実測値から決定する

// 平地を判定する閾値
const int CLIMB_FLAT_THRESHOLD = 700;

// 判定を安定させる時間
const unsigned long CLIMB_DETECT_TIME = 100;

// 山頂で停止する時間
const unsigned long CLIMB_TOP_WAIT_TIME = 200;

// 山頂確認用
static int topCheckCount = 0;
static unsigned long topCheckLastTime = 0;
static bool uphillExperienced = false;

// 山頂確認の設定
const unsigned long CLIMB_TOP_SAMPLE_INTERVAL = 50;
const int CLIMB_TOP_REQUIRED_SAMPLES = 8;

// 左右の傾きに応じて進行方向を補正する
void climbForwardWithSteering(int rollValue) {
  int steer = rollValue / CLIMB_STEER_DIVISOR;
  // 補正量を制限する
  steer = constrain(
    steer,
    -CLIMB_MAX_STEER,
    CLIMB_MAX_STEER
  );
  // 左右のモーター速度を調整する
  int leftSpeed = CLIMB_FORWARD_SPEED - steer;
  int rightSpeed = CLIMB_FORWARD_SPEED + steer;

  drive(leftSpeed, rightSpeed);
}

// 山登りの状態を初期化する
void resetClimbMission() {
  climbPhase = CLIMB_APPROACH;

  detectStartTime = 0;
  topWaitStartTime = 0;
  slopeDetected = false;

  // 山頂確認用の初期化
  topCheckCount = 0;
  topCheckLastTime = 0;
  uphillExperienced = false;

  previousPitchValue = 0;
}

// 山登りミッションを1周期進める
void updateClimbMission() {

  // ミッションが停止している場合はモーターを止める
  if (!robotEnabled || !hillMissionActive) {
    stopMotors();
    return;
  }

  // 加速度センサーの値を取得する
  int accelX;
  int accelY;
  // X軸とY軸を同時に取得する
  readAccelXY(accelX, accelY);

  // 平地を基準にした前後,左右方向の傾き
  int pitchValue = accelX - CLIMB_FLAT_X;
  int rollValue = accelY - CLIMB_FLAT_Y;

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
          // 斜面を検出したことを記録する
          uphillExperienced = true;
          climbPhase = CLIMB_UP;

          Serial1.println("ID=1,STATUS=HILL_STARTED");
        }

      } else {
        detectStartTime = 0;
      }

      break;

    case CLIMB_UP:{
      // 左右の傾きに応じて進行方向を補正する
      climbForwardWithSteering(rollValue);
      // 前回より前後方向の傾きが小さくなった量を計算する
      int pitchChange = previousPitchValue - pitchValue;
      // 登山中に傾きが急激に小さくなったら山頂候補とする
      if (
        uphillExperienced &&
        pitchChange >= CLIMB_TOP_CHANGE_THRESHOLD
      ) {
        stopMotors();

        detectStartTime = 0;
        topCheckCount = 0;
        topCheckLastTime = millis();

        climbPhase = CLIMB_TOP;
      }
      // 今回の傾きを次回の比較用に保存する
      previousPitchValue = pitchValue;
      break;
    }

    case CLIMB_TOP:
      // 山頂候補ではモーターを停止する
      stopMotors();
      // 一定間隔で傾きを確認する
      if (millis() - topCheckLastTime >= CLIMB_TOP_SAMPLE_INTERVAL) {
        topCheckLastTime = millis();
        // 前後左右とも平坦ならカウントする
        if (
          abs(pitchValue) <= CLIMB_FLAT_THRESHOLD &&
          abs(rollValue) <= CLIMB_FLAT_THRESHOLD
        ) {
          topCheckCount++;
        } else {
          // 傾きが大きくなったら山頂候補を取り消す
          topCheckCount = 0;
          // 登山を再開する
          climbPhase = CLIMB_UP;
        }
      }
      // 8回連続で平坦なら山頂と判断する
      if (topCheckCount >= CLIMB_TOP_REQUIRED_SAMPLES) {
        stopMotors();
        Serial1.println("ID=1,STATUS=HILL_TOP");
        // 下山開始
        slopeDetected = false;
        detectStartTime = 0;
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
