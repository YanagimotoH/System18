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

// color.inoで取得・校正したRGB値を、ColorMove.inoの色識別関数で分類する。
void updateColor() {
  // 通常はカラーセンサのキャリブレーション値を使う。
  getRGB(red_G, green_G, blue_G);

  // キャリブレーションを使わず生値の色比率で確認する場合は、上のgetRGB行を
  // コメントアウトし、次のブロックを有効にする。識別基準は8bit色比率の値。
  /*
  uint16_t rawR, rawG, rawB, clear;
  tcs.getRawData(&rawR, &rawG, &rawB, &clear);
  const uint32_t total = (uint32_t)rawR + rawG + rawB;
  if (total == 0) {
    red_G = green_G = blue_G = 0.0f;
  } else {
    red_G = (float)rawR * 255.0f / total;
    green_G = (float)rawG * 255.0f / total;
    blue_G = (float)rawB * 255.0f / total;
  }
  */

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
    robotEnabled = !robotEnabled;
    if (robotEnabled && captureHeadingReference()) {
      state = SEARCH;
      resetSearchPattern();
      Serial1.println("ID=1,STATUS=STARTED");
    } else if (robotEnabled) {
      robotEnabled = false;
      stopMotors();
      Serial1.println("ID=1,STATUS=HEADING_REFERENCE_FAILED");
    } else {
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
  if (millis() - lastSendTime < 200) {
    return;
  }
  lastSendTime = millis();

  Serial1.print("ID=");
  Serial1.print(ROBOT_ID);
  Serial1.print(",STATE=");
  Serial1.print(stateName());
  Serial1.print(",DIST=");
  Serial1.println(sensors.distanceCm);
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

// 入力を先に処理し、センサ判定・境界回避・状態別動作の順に1周期を進める。
void loop() {
    static bool lineAvoidLatched = false;
    checkButton();
    receiveCommand();
    if (!robotEnabled) {
        state = STOP;
        lineAvoidLatched = false;
        stopMotors();
        motors.setSpeeds(motorL, motorR);
        sendStatus();
        return;
    }
    sensors.distanceCm = distance();
    updateColor();
    sensors.obstacle = isObstacle(sensors.distanceCm);
    if (state == SEARCH) {
        sensors.targetDetected = detectTarget();
    }
    sensors.targetDistanceCm = sensors.targetDetected ? targetDistance() : -1;
    // ゴール直後は青ライン上から内側へ向き直して離れる。ラインを抜けたら通常検知に戻す。
    if (sensors.goalDetected && sensors.floorColor != MY_TEAM_COLOR) {
        sensors.goalDetected = false;
    }

    // 黒い外周と相手色は回避する。自チーム色はCARRYではゴール、SEARCHではライン離脱に使う。
    // 黒外周・相手色を優先して回避し、自陣色は探索中だけライン離脱に使う。
    if (state != STOP && state != GOAL &&
        (sensors.floorColor == FLOOR_BLACK ||
         isOpponentColor(sensors.floorColor) ||
         (state == SEARCH && sensors.floorColor == MY_TEAM_COLOR &&
          !sensors.goalDetected))) {
        if (!lineAvoidLatched) {
            lineAvoidLatched = true;
            state = AVOID;
        }
    } else {
        // ラインから離れたら次の接触を検出可能にする。
        lineAvoidLatched = false;
    }

    // 状態ごとの動作を一箇所で選び、最後にモーター出力と状態通知を反映する。
    switch (state) {
        case STOP:
            stopMotors();
            break;
        case SEARCH:
            search();
            break;
        case APPROACH:
            approachTarget();
            break;
        case AVOID:
            avoidObstacle();
            break;
        case CARRY:
            carryToGoal();
            break;
        case GOAL:
            checkGoal();
            break;
    }

    // CARRYへ入った瞬間だけ短く鳴らす。CARRY中は繰り返し鳴らさない。
    if (state == CARRY && previousState != CARRY) {
        tone(buzzerPin, 440, 50); // CARRY移行時に440Hzを50ms鳴らす
    }
    previousState = state;

    motors.setSpeeds(motorL, motorR);
    sendStatus();
}
