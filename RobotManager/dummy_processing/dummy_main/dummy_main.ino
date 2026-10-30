#include "RobotConfig.h"
#include <ZumoMotors.h>
#include <Pushbutton.h>

RobotState state = STOP;
SensorData sensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

int motorL = 0;
int motorR = 0;
bool robotEnabled = false;
bool buttonWasPressed = false;
RobotState previousState = STOP;

void checkButton() {
  bool buttonPressed = button.isPressed();

  // 押された瞬間だけ実行し、押し続けても状態が連続で切り替わらないようにする。
  if (buttonPressed && !buttonWasPressed) {
    robotEnabled = !robotEnabled;

    if (robotEnabled) {
      state = SEARCH;
      resetSearchPattern();
      Serial1.println("ID=1,STATUS=STARTED");
    } else {
      stopMotors();
      Serial1.println("ID=1,STATUS=STOPPED");
    }
  }

  buttonWasPressed = buttonPressed;
}

void receiveCommand() {
  static String command;

  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '\n' || c == '\r') {
      command.trim();

      if (command == "START") {
        robotEnabled = true;
        state = SEARCH;
        resetSearchPattern();
        Serial1.println("ID=1,STATUS=STARTED");
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

// Processing表示確認用のダミーデータ送信
void sendDummyData() {
    static unsigned long lastSendTime = 0;
    static unsigned long lastModeChange = 0;
    static int modeIndex = 0;

    const char* dummyStates[] = {
        "STOP",
        "SEARCH",
        "APPROACH",
        "AVOID",
        "CARRY",
        "GOAL"
    };

    const int dummyDistances[] = {
        -1, 80, 35, 20, 12, 5
    };

    unsigned long now = millis();

    // 2秒ごとに表示する状態を切り替える
    if (now - lastModeChange >= 2000) {
        lastModeChange = now;
        modeIndex = (modeIndex + 1) % 6;
    }

    // 200msごとにデータを送信する
    if (now - lastSendTime < 200) {
        return;
    }
    lastSendTime = now;

    Serial1.print("ID=");
    Serial1.print(ROBOT_ID);
    Serial1.print(",STATE=");
    Serial1.print(dummyStates[modeIndex]);
    Serial1.print(",DIST=");
    Serial1.println(dummyDistances[modeIndex]);
}

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    motors.setSpeeds(0, 0);
}

void loop() {
    // 実際のモーター制御は行わない
    motors.setSpeeds(0, 0);

    // Processingにダミーデータを送信
    sendDummyData();
}