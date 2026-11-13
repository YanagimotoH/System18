#include "RobotConfig.h"
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <Adafruit_TCS34725.h>

// color.ino / ColorMove.ino が共有するセンサ値・旧move()用変数。
// Mainの状態機械ではmove()を呼ばず、ColorMove.inoのidentify_color()だけ使う。
extern Adafruit_TCS34725 tcs;
float red_G = 0.0f;
float green_G = 0.0f;
float blue_G = 0.0f;
int mode_G = 0;
int motorL_G = 0;
int motorR_G = 0;

RobotState state = STOP; // ロボットの状態
SensorData sensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

int motorL = 0;
int motorR = 0;
bool robotEnabled = false;
uint8_t startupStage = 0; // 0:色センサ、1:方位校正、2:実行待ち
bool buttonWasPressed = false;
RobotState previousState = STOP;

/*Climb.inoで使う変数*/
// Processingで山登り担当に選ばれているか
// true  : 山登り担当
// false : 通常探索担当
bool hillAssigned = false;

// この試合中に山登りを実行したか
// 一度登山を完了したらtrueにする
// 二度目の山登りを防ぐために使用する
bool hillUsed = false;

// 現在、山登りミッションを実行しているか
// 登坂開始から下山完了までtrueにする
bool hillMissionActive = false;


// color.inoで取得・校正したRGB値を、ColorMove.inoの色識別関数で分類する。
void updateColor() {
  // 通常はカラーセンサのキャリブレーション値を使う。
  getRGB(red_G, green_G, blue_G);

  // キャリブレーションを使わず生値の色比率で確認する場合は、上のgetRGB行を
  // コメントアウトし、次のブロックを有効にする。識別基準は8bit色比率の値。
  
  // uint16_t rawR, rawG, rawB, clear;
  // tcs.getRawData(&rawR, &rawG, &rawB, &clear);
  // const uint32_t total = (uint32_t)rawR + rawG + rawB;
  // if (total == 0) {
  //   red_G = green_G = blue_G = 0.0f;
  // } else {
  //   red_G = (float)rawR * 255.0f / total;
  //   green_G = (float)rawG * 255.0f / total;
  //   blue_G = (float)rawB * 255.0f / total;
  // }
  
  sensors.colorRed = red_G;
  sensors.colorGreen = green_G;
  sensors.colorBlue = blue_G;

  if (identify_color(118, 54, 56)) {
    sensors.floorColor = FLOOR_RED;
  } else if (identify_color(39, 69, 101)) {
    sensors.floorColor = FLOOR_BLUE;
  } else if (identify_color(29, 40, 39)) {
    sensors.floorColor = FLOOR_BLACK;
  } else {
    sensors.floorColor = FLOOR_UNKNOWN;
  }
}

bool isOpponentColor(FloorColor color) {
  return (MY_TEAM_COLOR == FLOOR_BLUE && color == FLOOR_RED) ||
         (MY_TEAM_COLOR == FLOOR_RED && color == FLOOR_BLUE);
}

// 起動段階に応じて色校正・方位校正を行い、その後はボタンで走行を切り替える。
void checkButton() {
  bool buttonPressed = button.isPressed();

  // 押された瞬間だけ実行し、押し続けても状態が連続で切り替わらないようにする。
  if (buttonPressed && !buttonWasPressed) {
    if (startupStage == 0) {
      stopMotors();
      motors.setSpeeds(0, 0);
      CalibrationColorSensor();
      startupStage = 1;
      Serial1.println("ID=1,STATUS=COLOR_READY");
      buttonWasPressed = button.isPressed();
      return;
    }
    if (startupStage == 1) {
      stopMotors();
      motors.setSpeeds(0, 0);
      const bool calibrated = calibrateCompass();
      startupStage = 2; // 校正が不完全なら3回目に生値方位へフォールバック
      Serial1.println(calibrated
                          ? "ID=1,STATUS=COMPASS_CALIBRATED"
                          : "ID=1,STATUS=COMPASS_CAL_FALLBACK");
      buttonWasPressed = button.isPressed();
      return;
    }
    // 走行開始・停止処理
    // ロボットが停止中の場合は走行を開始する
    if (!robotEnabled) {
        // 現在の向きを基準方位として記録する
        if (captureHeadingReference()) {
            // 走行を許可する
            robotEnabled = true;
            // 山登り担当かどうかを確認する
            if (hillAssigned && !hillUsed) {
            // 山登りミッションを実行状態にする
            hillMissionActive = true;
            // この試合では山登りを使用済みにする
            hillUsed = true;
            // 山登り開始時の状態を設定する
            state = SEARCH;
            } else {
            // 山登り担当ではない場合は通常探索を行う
            hillMissionActive = false;
            // 通常探索を開始する
            state = SEARCH;
            // 探索パターンを初期化する
            resetSearchPattern();
            }
            // Processingに走行開始を通知する
            Serial1.println("ID=1,STATUS=STARTED");
        } else {
            // 方位基準の取得に失敗した場合は走行させない
            robotEnabled = false;
            stopMotors();
            Serial1.println("ID=1,STATUS=HEADING_REFERENCE_FAILED");
        }
    } else {
    // 走行中にボタンを押した場合は停止する
    robotEnabled = false;
    // 山登り処理も停止状態にする
    hillMissionActive = false;
    stopMotors();
    Serial1.println("ID=1,STATUS=STOPPED");
    }
  }

  buttonWasPressed = buttonPressed;
}

// Serial1の改行区切りコマンドを読み、START/STOPに応じて状態を更新する。
void receiveCommand() {
  static String command;

  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '\n' || c == '\r') {
      command.trim();
        // 山登り担当の設定
        // ProcessingからHILL_ONを受信した場合
        if (command == "HILL_ON") {
            // 走行開始前で、まだ山登りを実行していない場合だけ設定
            if (!robotEnabled && !hillUsed) {
                hillAssigned = true;
                Serial1.println("ID=1,STATUS=HILL_ASSIGNED");
            }
        } else if (command == "HILL_OFF") { // ProcessingからHILL_OFFを受信した場合
        // 走行開始前で、まだ山登りを実行していない場合だけ解除
            if (!robotEnabled && !hillUsed) {
                hillAssigned = false;
                Serial1.println("ID=1,STATUS=HILL_UNASSIGNED");
            }
        }

      if (command == "START") {
        if (startupStage >= 2 && captureHeadingReference()) {
          robotEnabled = true;
          state = SEARCH;
          resetSearchPattern();
          Serial1.println("ID=1,STATUS=STARTED");
        } else {
          Serial1.println("ID=1,STATUS=CALIBRATE_FIRST");
        }
      } else if (command == "STOP") {
        robotEnabled = false;
        state = STOP;
        stopMotors();
        Serial1.println("ID=1,STATUS=STOPPED");
      }

      command = "";
    } else {
      command += c;
    }
  }
}

// 通信表示用に現在の状態を文字列へ変換する。
const char* stateName() {
  switch (state) {
    case STOP: return "STOP";
    case SEARCH: return "SEARCH";
    case APPROACH: return "APPROACH";
    case AVOID: return "AVOID";
    case CARRY: return "CARRY";
    case GOAL: return "GOAL";
    default: return "UNKNOWN";
  }
}

// ロボットID・状態・距離を200ms間隔でSerial1へ送る。
void sendStatus() {
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 200) return;
  lastSendTime = millis();

  Serial1.print("ID=");
  Serial1.print(ROBOT_ID);
  Serial1.print(",STATE=");
  Serial1.print(hillMissionActive ? "CLIMB" : stateName());
  Serial1.print(",DIST=");
  Serial1.println(sensors.distanceCm);
  // カラーセンサーのRGB値を送信
  Serial1.print(",R=");
  Serial1.print(sensors.colorRed);
  Serial1.print(",G=");
  Serial1.print(sensors.colorGreen);
  Serial1.print(",B=");
  Serial1.println(sensors.colorBlue);
}

// シリアル、方位・超音波センサ、モーターを初期化する。
void setup() {
    // 通信・地磁気センサの初期化
    Serial.begin(9600);
    Serial1.begin(9600);
    setupCompass();
    // 超音波センサのピン設定
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    // 走行命令の初期化
    stopMotors();
}

// メインループ
// 入力処理 → センサー取得 → 山登り判定 → 通常動作
void loop() {
    // 黒ラインなどに接触したとき、同じ場所で
    // 回避処理が何度も開始されるのを防ぐ
    static bool lineAvoidLatched = false;
    // ボタンとProcessingからの命令を処理
    checkButton();
    receiveCommand();
    /**/
    // 加速度センサーのX軸を100msごとに送信する。
    // 加速度センサーの診断情報を100msごとに送信する。
    static unsigned long lastAccelSend = 0;

    if (millis() - lastAccelSend >= 100) {
        lastAccelSend = millis();
        sendAccelDebug();
    }
    /**/

    // ロボットが停止中の場合
    if (!robotEnabled) {
        state = STOP;
        lineAvoidLatched = false;
        stopMotors();
        motors.setSpeeds(motorL, motorR);
        sendStatus();
        return;
    }

    // 超音波センサーから距離を取得
    sensors.distanceCm = distance();
    // カラーセンサーから床の色を取得
    updateColor();
    // 障害物が存在するか判定
    sensors.obstacle = isObstacle(sensors.distanceCm);

    // 山登り担当のロボットがミッション実行中なら、
    // 通常探索ではなく山登り専用処理を実行する
    if (hillMissionActive) {
        // 登坂・頂上探索・下山などを進める
        updateClimbMission();
        // 山登り処理で決めたモーター速度を反映
        motors.setSpeeds(motorL, motorR);
        // Processingへ現在の状態を送信
        sendStatus();
        // 通常探索の処理には進まない
        return;
    }
    // 通常探索時の対象物検出
    if (state == SEARCH) {
        // コップなどの対象物を検出
        sensors.targetDetected = detectTarget();
    }
    // 対象物が検出されていれば距離を取得
    // 検出されていなければ-1にする
    sensors.targetDistanceCm = sensors.targetDetected ? targetDistance() : -1;

    // ゴール直後に青ライン上から内側へ向き直して離れる。
    // ラインを抜けたら通常の検知に戻す。
    if (sensors.goalDetected && sensors.floorColor != MY_TEAM_COLOR) {
        sensors.goalDetected = false;
    }
    // 黒い外周や相手色に接触した場合は回避する。
    // 自チーム色はSEARCH中のみライン離脱に使用する。
    if (state != STOP && state != GOAL &&
        (sensors.floorColor == FLOOR_BLACK || isOpponentColor(sensors.floorColor) ||
         (state == SEARCH && sensors.floorColor == MY_TEAM_COLOR && !sensors.goalDetected))) {
        // まだ回避処理を開始していない場合
        if (!lineAvoidLatched) {
            lineAvoidLatched = true;
            // 回避状態へ変更
            state = AVOID;
        }
    } else {
        // ラインから離れたら次の接触を検出可能にする
        lineAvoidLatched = false;
    }
    // 通常動作の実行
    switch (state) {
        case STOP:
            // モーターを停止
            stopMotors();
            break;
        case SEARCH:
            // 通常探索
            search();
            break;
        case APPROACH:
            // 対象物へ接近
            approachTarget();
            break;
        case AVOID:
            // 障害物やラインを回避
            avoidObstacle();
            break;
        case CARRY:
            // コップを運搬してゴールへ向かう
            carryToGoal();
            break;
        case GOAL:
            // ゴール判定
            checkGoal();
            break;
    }
    // CARRYへ移行した瞬間だけ短く鳴らす
    if (state == CARRY && previousState != CARRY) {
        tone(buzzerPin, 440, 50);
    }
    // 現在の状態を保存
    previousState = state;
    // モーター出力と状態通知
    motors.setSpeeds(motorL, motorR);
    sendStatus();
}
