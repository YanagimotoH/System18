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

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    stopMotors();
}

void loop() {
    checkButton();
    receiveCommand();
    if (!robotEnabled) {
        state = STOP;
        stopMotors();
        motors.setSpeeds(motorL, motorR);
        sendStatus();
        return;
    }
    sensors.distanceCm = distance();
    sensors.obstacle = isObstacle(sensors.distanceCm);
    if (state == SEARCH) {
        sensors.targetDetected = detectTarget();
    }
    sensors.targetDistanceCm = sensors.targetDetected ? targetDistance() : -1;

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
