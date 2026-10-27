#include "Arduino.h"
#include "Wire.h"
#include "RobotConfig.h"

HostSerial Serial;
HostSerial Serial1;
HostWire Wire;
unsigned long hostMillis = 0;
int hostDistanceCm = -1;
int hostFloor = FLOOR_UNKNOWN;
float hostHeading = 0.0f;
int hostCalibrationMotorL = 0;
int hostCalibrationMotorR = 0;

extern float red_G;
extern float green_G;
extern float blue_G;

float hostHeadingDeg() { return hostHeading; }

void hostAdvanceTime(unsigned long value) {
  hostMillis += value;
  const float left = hostCalibrationMotorL * 0.075f;
  const float right = hostCalibrationMotorR * 0.075f;
  const float angular = (right - left) / 8.5f;
  hostHeading += angular * value * 180.0f / 3.14159265358979323846f / 1000.0f;
}

void CalibrationColorSensor() {}

void getRGB(float& red, float& green, float& blue) {
  switch (hostFloor) {
    case FLOOR_RED: red = 118; green = 54; blue = 56; break;
    case FLOOR_BLUE: red = 39; green = 69; blue = 101; break;
    case FLOOR_BLACK: red = 29; green = 40; blue = 39; break;
    default: red = green = blue = 0; break;
  }
}
