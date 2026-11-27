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

// ---- ロボット別の設定（3台分） ----
// 書き込むロボットに合わせて ROBOT_NUMBER を 1～3 のどれかに変更する。
#define ROBOT_NUMBER 3
const uint8_t ROBOT_ID = ROBOT_NUMBER; // 通信で使用するロボットID

// 方位センサの校正はこの時間内に成功しなければ失敗とみなし、固定値を使う。
const unsigned long CALIBRATION_TIMEOUT_MS = 10000UL;
const uint16_t COMPASS_CAL_MIN_SAMPLES = 200;     // 方位校正の最低サンプル数（30ms間隔で約6秒）
const int32_t COMPASS_CAL_MIN_RANGE = 50;         // 方位校正でXY各軸に必要な最小の変化幅。実機で調整

// 校正に失敗したときに使うロボット別の固定値。
//   FIXED_ROTATE_180_TIME_MS : ROTATE_SPEEDで180度旋回するのにかかる時間[ms]
//                              （方位校正に失敗したときの固定方位モード、および旋回時間全般に使用）
// ★下記の数値は仮の値。各ロボットで実測した値に置き換えること。
#if ROBOT_NUMBER == 1
  #define FIXED_ROTATE_180_TIME_MS 2000UL
#elif ROBOT_NUMBER == 2
  #define FIXED_ROTATE_180_TIME_MS 2000UL
#elif ROBOT_NUMBER == 3
  #define FIXED_ROTATE_180_TIME_MS 2000UL
#else
  #error "ROBOT_NUMBER must be 1, 2 or 3"
#endif

// 超音波センサのピン番号
const int trig = 2; // 超音波センサのトリガーピン
const int echo = 4; // 超音波センサのエコーピン

const int buzzerPin = 3; // ブザーを接続するピン

// 距離・速度の設定
const int OBSTACLE_LIMIT_CM = 15; // 障害物と判定する距離の閾値
const int APPROACH_LIMIT_CM = 3; // 目標に接近する距離の閾値
const int TARGET_DETECTION_RANGE_CM = 40; // 0～100cm内の物体だけカップ候補として確認
const int FORWARD_SPEED = 170; // 前進速度（探索・接近・後退・ライン脱出で共通）
const int ROTATE_SPEED = 150; // 方位旋回速度を少し上げる
const unsigned long ROTATE_180_TIME_MS = FIXED_ROTATE_180_TIME_MS; // 180度回転時間（ロボット別の固定値を使用）
// 基準方位を記録するときはロボットをコース北へ向ける。以後は北=0°の相対方位。
// 東=90°、南=180°、西=270°。
const float NORTH_HEADING_DEG = 0.0f;
const float EAST_HEADING_DEG = 90.0f;
const float SOUTH_HEADING_DEG = 180.0f;
const float WEST_HEADING_DEG = 270.0f;
// 運搬(CARRY)で向かうゴール方位。ここだけ変更し、上の北/東/南/西の固定値から選ぶ。
// 地磁気センサの校正に成功した場合も、失敗して固定方位モードになった場合も同じ値を使う。
#if ROBOT_NUMBER == 1
const float CARRY_GOAL_HEADING_DEG = EAST_HEADING_DEG;  // 東 90°
#elif ROBOT_NUMBER == 2
const float CARRY_GOAL_HEADING_DEG = WEST_HEADING_DEG;  // 西 270°
#elif ROBOT_NUMBER == 3
const float CARRY_GOAL_HEADING_DEG = SOUTH_HEADING_DEG; // 南 180°
#endif
const float HEADING_TOLERANCE_DEG = 8.0f; // 1回の方位読み取りで旋回完了とする許容誤差
const unsigned long SWEEP_150_TIME_MS =
    ROTATE_180_TIME_MS * 150UL / 180UL; // 約150度旋回する時間
const int BACKWARD_SPEED = FORWARD_SPEED; // 後退速度（前進速度と同じ）

// ---- 地磁気センサの校正に失敗した場合の固定方位モード ----
// 方位校正がCALIBRATION_TIMEOUT_MS(10秒)以内に成功しなければ、
// 磁気センサを使わず、旋回時間から現在方位を推定する固定方位モードへ切り替える。
// このモードでは、基準方位を記録した向きを北(0°)とみなし、上の北/東/南/西の固定値と
// CARRY_GOAL_HEADING_DEG をそのまま使う。旋回量はROTATE_180_TIME_MSから換算するため、
// 実機でROTATE_SPEEDでの180度旋回時間を合わせておくこと。
const float FIXED_HEADING_DEG_PER_MS =
    180.0f / ROTATE_180_TIME_MS; // 旋回速度[度/ms]。ROTATE_180_TIME_MSから算出
const unsigned long FIXED_HEADING_MAX_STEP_MS = 300UL; // これ以上空いた旋回指令は中断とみなし加算しない
void enableFixedHeadingMode(); // 校正失敗時に、旋回時間で方位を推定する固定方位モードへ切り替える
bool isFixedHeadingMode(); // 固定方位モード（磁気センサ不使用）かを返す
void fixedHeadingOnDrive(); // 固定方位モードで旋回が中断された分を方位推定へ反映する（drive()から呼ぶ）

// ---- ライン接触時の回避 ----
// 黒・赤・青のラインを踏んだら、AVOID_BACK_MS 後退してから180度旋回し、状態ごとの続きへ戻る。
// （自チーム色のゴールは対象外。将来のライントレース追加を考え、動作は単純にしてある）
const unsigned long AVOID_BACK_MS = 1000UL; // ライン検出後の後退時間
const unsigned long AVOID_TURN_TIMEOUT_MS = ROTATE_180_TIME_MS * 2UL; // 180度旋回の上限時間
// 例外処理: 回避(AVOID)がこの時間を超えたら、状態に関わらず強制的に探索(SEARCH)へ抜ける。
const unsigned long AVOID_MAX_MS = AVOID_BACK_MS + AVOID_TURN_TIMEOUT_MS + 500UL;
// 回避が終わってからAVOID_REENTRY_MS以内に再びラインを踏むことがAVOID_REENTRY_MAX回続いたら、
// 回避後は運搬・ゴール処理の続きに戻らず探索(SEARCH)へ抜ける（同じラインでの無限ループ防止）。
const unsigned long AVOID_REENTRY_MS = 3000UL;
const uint8_t AVOID_REENTRY_MAX = 3;

// ---- 探索 ----
// 360度回転して確認した後、探索の前進方向を中心に左右この角度の範囲でランダムな向きへ前進する。
const int SEARCH_FORWARD_SPREAD_DEG = 30;
bool isLineContact(RobotState s); // 状態sでラインを踏んだとみなす床色かを返す
void beginAvoid(RobotState from); // ライン接触の回避を開始する（回避後はfromの続きへ戻る）

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
void initCompassHardware(); // 基準方位を保持したままLSM303を再検出する
void setupCompass(); // 方位センサを初期化する
bool calibrateCompass(); // 方位センサの磁気範囲を校正する
bool captureHeadingReference(); // 現在の方位を相対方位の基準として記録する
bool compassIsReady(); // 方位センサが初期化済みか返す
float readHeadingDeg(); // 基準方位からの相対方位を度で取得する
int getHeadingRotationSign(); // 方位誤差を減らすモーター旋回方向を返す
bool turnToHeading(float targetDeg, float toleranceDeg = HEADING_TOLERANCE_DEG); // 指定方位への旋回が完了したか判定する

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

void sendStatus(); // ロボットID、状態、距離をシリアル送信する
void receiveCommand(); // シリアルからSTART/STOPコマンドを受信する

// 運搬: 走行開始時の向きを0°として、CARRY_GOAL_HEADING_DEGへ正確に向いてから前進する。
const float CARRY_HEADING_TOLERANCE_DEG = HEADING_TOLERANCE_DEG; // 旋回完了とみなす許容誤差
const float CARRY_REALIGN_DEG = 20.0f; // 前進中にゴール方位からこれ以上ずれたら止まって向き直す
// カップが前方(APPROACH_LIMIT_CM以内)にない状態がこの回数続いたら、カップを見失ったとみなして探索へ戻る。
const uint8_t CUP_LOST_SAMPLES = 5;
const unsigned long GOAL_CONFIRM_MS = 300UL; // ゴール検出後、停止してカップがあることを確認する時間
bool cupLost(); // 前方にカップがない状態が続いたか（運搬・ゴール処理中の判定）
void resetCupLost(); // カップ見失い判定のカウントを初期化する

// 以下山登り関係
bool readAccelXY(int &x, int &y); // 成功時だけ加速度のX,Yを更新する

extern bool hillAssigned; // 山登り担当に設定されているか
extern bool hillUsed; // この試合ですでに山登りを試みたか
extern bool hillMissionActive; // 山登りミッションの実行中か

void updateClimbMission(); // 山登りミッションを1周期進める
void sendAccelDebug();