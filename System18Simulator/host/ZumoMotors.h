#pragma once
extern int hostCalibrationMotorL;
extern int hostCalibrationMotorR;
class ZumoMotors {
 public:
  void setSpeeds(int left, int right) { hostCalibrationMotorL = left; hostCalibrationMotorR = right; }
};
