#include "Arduino.h"
#include "RobotConfig.h"
#include <iostream>
#include <sstream>
#include <string>

void approachTarget();
#include "Main.ino"
#include "ColorMove.ino"
#include "Exploration.ino"
#include "Movement.ino"
#include "Ultrasound.ino"
#include "Compass.ino"

extern int hostFloor;
extern float hostHeading;
extern unsigned long hostMillis;
extern int hostDistanceCm;
extern bool hostButtonPressed;

int main() {
  setup();

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    unsigned long frameTime = 0;
    int floor = FLOOR_UNKNOWN;
    std::string command;
    if (!(input >> frameTime >> hostDistanceCm >> floor >> hostHeading >> command)) continue;

    hostMillis = std::max(hostMillis, 1000UL + frameTime);
    hostFloor = floor;
    const uint8_t stageBeforeInput = startupStage;
    if (command != "-" && command != "BUTTON") Serial1.inject(command + "\n");
    hostButtonPressed = command == "BUTTON";
    loop();
    hostButtonPressed = false;

    const char* event = "-";
    if (command == "BUTTON") {
      if (stageBeforeInput == 0 && startupStage == 1) event = "COLOR_READY";
      else if (stageBeforeInput == 1)
        event = startupStage == 2 ? "COMPASS_CALIBRATED" : "COMPASS_CAL_FAILED";
      else if (stageBeforeInput >= 2) event = robotEnabled ? "STARTED" : "STOPPED";
    } else if (command == "START") {
      event = robotEnabled ? "STARTED" : "CALIBRATE_FIRST";
    } else if (command == "STOP") {
      event = "STOPPED";
    }

    std::cout << static_cast<int>(state) << ',' << motorL << ',' << motorR << ','
              << robotEnabled << ',' << sensors.targetDetected << ','
              << sensors.distanceCm << ',' << static_cast<int>(startupStage) << ','
              << readHeadingDeg() << ',' << hostHeading << ',' << event << std::endl;
  }
  return 0;
}
