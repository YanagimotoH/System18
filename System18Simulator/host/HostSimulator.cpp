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

int main() {
  setup();
  calibrateCompass();
  startupStage = 2;

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    unsigned long frameTime = 0;
    int floor = FLOOR_UNKNOWN;
    std::string command;
    if (!(input >> frameTime >> hostDistanceCm >> floor >> hostHeading >> command)) continue;

    hostMillis = 1000 + frameTime;
    hostFloor = floor;
    if (command != "-") Serial1.inject(command + "\n");
    loop();
    std::cout << static_cast<int>(state) << ',' << motorL << ',' << motorR << ','
              << robotEnabled << ',' << sensors.targetDetected << ','
              << sensors.distanceCm << std::endl;
  }
  return 0;
}
