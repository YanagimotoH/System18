#pragma once
#include <Arduino.h>

// ロボットの状態を表す列挙型
enum RobotState {
    STOP,     // 停止中
    SEARCH,   // 対象を探索中
    APPROACH, // 対象へ接近中
    AVOID,    // 障害物や境界を回避中
    CARRY,    // 対象を運搬中
    GOAL      // ゴール処理中
};

// 床面色。自チーム色はこの宣言だけを変更して設定する。
enum FloorColor {
    FLOOR_UNKNOWN, // 色を判定できない
    FLOOR_BLACK,   // 黒い外周
    FLOOR_RED,     // 赤チームの領域
    FLOOR_BLUE     // 青チームの領域
};
const FloorColor MY_TEAM_COLOR = FLOOR_BLUE; // ロボットの所属チーム色

// センサーのデータを格納する構造体
struct SensorData {
    int distanceCm;             // 超音波センサの測定距離（cm）
    int targetDistanceCm;       // 対象までの距離（cm）、未検出時は-1
    bool obstacle;              // 障害物判定結果
    bool targetDetected;        // 対象候補の検出結果
    bool goalDetected;          // ゴール検出結果
    FloorColor floorColor;      // 現在の床面色
    float colorRed;             // 色センサの赤成分
    float colorGreen;           // 色センサの緑成分
    float colorBlue;            // 色センサの青成分
};

const uint8_t ROBOT_ID = 1; // 通信で使用するロボットID

// 超音波センサのピン番号
const int trig = 2; // 超音波センサのトリガーピン
const int echo = 4; // 超音波センサのエコーピン

const int buzzerPin = 3; // ブザーを接続するピン

// 距離・速度の設定
const int OBSTACLE_LIMIT_CM = 15; // 障害物と判定する距離の閾値
const int APPROACH_LIMIT_CM = 8; // 目標に接近する距離の閾値
const int TARGET_DETECTION_RANGE_CM = 100; // 0～100cm内の物体だけカップ候補として確認
const int FORWARD_SPEED = 100; // 前進速度
const int ROTATE_SPEED = 150; // 回転速度
const unsigned long ROTATE_180_TIME_MS = 2000; // 180度回転時間の初期値。実機で調整
// 1回目のセンサ準備時に向いていた方向を0°、その反対を180°として扱う。
const float NORTH_HEADING_DEG = 0.0f; // 基準方向の相対方位
const float SOUTH_HEADING_DEG = 180.0f; // 基準方向と反対の方位
const float HEADING_TOLERANCE_DEG = 2.0f; // 方位合わせの許容誤差
const unsigned long SWEEP_150_TIME_MS =
    ROTATE_180_TIME_MS * 150UL / 180UL; // 約150度旋回する時間
const int BACKWARD_SPEED = 80; // 障害物回避時の速度

extern RobotState state; // 現在のロボットの状態
extern SensorData sensors; // センサーのデータ

// モーターの速度を格納する変数
extern int motorL; // 左モーターの速度
extern int motorR; // 右モーターの速度

extern bool robotEnabled; // ロボットが有効かどうかを示すフラグ
void resetSearchPattern(); // 探索パターンと探索用の状態を初期化する
void resetCarryMotion(); // 運搬動作の状態を初期化する
void CalibrationColorSensor(); // 色センサの測定範囲を校正する
void getRGB(float& r, float& g, float& b); // 校正値を使ってRGB値を取得する
void updateColor(); // 色センサ値を取得して床面色を更新する
int identify_color(int red, int green, int blue); // 指定RGB値に近いか判定する
bool isOpponentColor(FloorColor color); // 指定色が相手チーム色か判定する
void setupCompass(); // 方位センサを初期化する
bool calibrateCompass(); // 方位センサの磁気範囲を校正する
bool captureHeadingReference(); // 現在の方位を相対方位の基準として記録する
bool compassIsReady(); // 方位センサが初期化済みか返す
float readHeadingDeg(); // 基準方位からの相対方位を度で取得する
bool turnToHeading(float targetDeg); // 指定方位への旋回が完了したか判定する

// Movement.inoの関数の宣言
void drive(int leftSpeed, int rightSpeed); // 左右モーターに速度を設定する（左、右）
void stopMotors(); // 左右モーターを停止する
void moveForward(int speed); // 指定速度で前進する
void moveBackward(int speed); // 指定速度で後退する
void rotateInPlace(int speed); // 指定速度でその場旋回する

// Ultrasound.inoの関数の宣言
int distance(); // 超音波センサから距離をcm単位で取得する
bool isObstacle(int distanceCm); // 距離が障害物判定範囲内か返す
bool detection(); // 現在の距離が対象検出範囲内か返す
void transport(); // 対象がなければ前進し、検出時に停止する

// Exploration.inoの関数の宣言
void search(); // 探索走行と対象候補の確認を行う
void approachTarget(); // 対象候補へ接近する
bool detectTarget(); // 対象候補を検出したか返す
int targetDistance(); // 対象候補までの距離をcm単位で返す


// Exploration.inoの関数の宣言
void avoidObstacle(); // 段階的な後退・旋回で障害物を回避する

// Movement.inoの関数の宣言
void grabCup(); // カップ保持処理を行う（保持機構は未実装）
bool detectGoal(); // 自チーム色を確認してゴール到達を判定する
void checkGoal(); // ゴール到達後の処理を進める
void carryToGoal(); // カップをゴール方向へ運ぶ
void resumeSearch(); // 運搬後に探索状態へ戻す

// Communication.inoの関数の宣言
void sendStatus(); // ロボットID、状態、距離をシリアル送信する
void receiveCommand(); // シリアルからSTART/STOPコマンドを受信する

