#pragma once
#include <Arduino.h>

// ロボットの状態を表す列挙型
enum RobotState {
    STOP,
    SEARCH,
    APPROACH,
    AVOID,
    CARRY,
    GOAL
};

// センサーのデータを格納する構造体
struct SensorData {
    int distanceCm;
    int targetDistanceCm;
    bool obstacle;
    bool targetDetected;
    bool goalDetected;
};

const uint8_t ROBOT_ID = 1; // ロボットのID

// 超音波センサのピン番号
const int trig = 2; // 超音波センサのトリガーピン
const int echo = 4; // 超音波センサのエコーピン

const int buzzerPin = 3; //ブザーは 3 番ピン

// 距離・速度の設定
const int OBSTACLE_LIMIT_CM = 15; // 障害物と判定する距離の閾値
const int APPROACH_LIMIT_CM = 8; // 目標に接近する距離の閾値
const int TARGET_DETECTION_RANGE_CM = 70; // 探索時に物体候補を確認する距離
const int OBJECT_MOTION_THRESHOLD_CM = 3; // 動体と判定する距離変化
const unsigned long OBJECT_MOTION_WINDOW_MS = 700; // 動きを確認する時間
const unsigned long OBJECT_SAMPLE_INTERVAL_MS = 100; // 距離を確認する間隔
const unsigned long TARGET_CONFIRM_TIME_MS = 300; // 旋回中に候補が視界に残ることを確認する時間
const unsigned long TARGET_SETTLE_TIME_MS = 150; // 停止後、距離が安定するまで待つ時間
const int FORWARD_SPEED = 100; // 前進速度
const int ROTATE_SPEED = 130; // 回転速度
const unsigned long ROTATE_180_TIME_MS = 1000; // 180度回転時間の初期値。実機で調整
const unsigned long SWEEP_150_TIME_MS =
    ROTATE_180_TIME_MS * 150UL / 180UL; // 約150度旋回する時間
const unsigned long CARRY_MIN_FORWARD_TIME_MS = 400; // 離脱後、最低限前進する時間
const int CARRY_CLEAR_DISTANCE_CM = 40; // 運搬中に前方が開けたとみなす距離
const int BACKWARD_SPEED = 80; // 障害物回避時の速度

extern RobotState state; // 現在のロボットの状態
extern SensorData sensors; // センサーのデータ

// モーターの速度を格納する変数
extern int motorL; // 左モーターの速度
extern int motorR; // 右モーターの速度

extern bool robotEnabled; // ロボットが有効かどうかを示すフラグ
void resetSearchPattern(); // 探索パターンをリセットする関数
void resetCarryMotion(); // カップ到達後の動作をリセットする関数

// Movement.inoの関数の宣言
void drive(int leftSpeed, int rightSpeed); // 左右のモーターの速度を設定する関数
void stopMotors(); // モーターを停止する関数
void moveForward(int speed); // 前進する関数
void moveBackward(int speed); // 後退する関数
void rotateInPlace(int speed); // その場で回転する関数

// Ultrasound.inoの関数の宣言
int distance(); // 超音波センサーから距離を取得する関数
bool isObstacle(int distanceCm); // 障害物があるかどうかを判定する関数
bool detection(); // 超音波センサーで障害物を検出する関数
void transport(); // 障害物がない間の基本走行を行う関数

// Exploration.inoの関数の宣言
void search(); // 探索を行う関数
void approach(); // 目標に接近する関数
bool detectTarget(); // 目標を検出する関数
int targetDistance(); // 目標までの距離を取得する関数


// 以下仮
// ObstacleAvoidance.inoの関数の宣言
void avoidObstacle(); // 障害物を回避する関数

// CupHandling.inoの関数の宣言
void grabCup(); // カップを掴む関数
bool isCupHeld(); // カップを保持しているかどうかを判定する関数
void releaseCup(); // カップを放す関数

// Goal.inoの関数の宣言
bool detectGoal(); // ゴールを検出する関数
bool isGoalStatisfied(); // ゴール条件を満たしているかどうかを判定する関数
void checkGoal(); // ゴール条件を確認する関数
void carryToGoal(); // ゴールまでカップを運ぶ関数
void resumeSearch(); // 探索を再開する関数
void addScore(int points); // スコアを加算する関数

// Communication.inoの関数の宣言
void sendStatus(); // 状態を送信する関数
void receiveCommand(); // コマンドを受信する関数

