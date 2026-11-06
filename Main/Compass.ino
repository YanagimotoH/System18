#include <Wire.h>
#include <LSM303.h>
#include <math.h>
#include "RobotConfig.h"
#include <ZumoMotors.h>

// 地磁気センサによる方位制御の基礎処理。
// 山登りと山上のカップ回収動作はここでは未実装。
// 現在はセンサ校正、方位取得、指定方位への旋回だけを提供する仮の基盤。（統合して欲しい個所）
extern ZumoMotors motors;
static LSM303 compass;
static bool compassReady = false;
static bool compassCalibrated = false;
static float headingReferenceDeg = 0.0f;
static int16_t magMinX, magMinY, magMaxX, magMaxY;
static int rotationHeadingSign = 1;

// 地磁気センサを初期化し、初期校正前の状態にする。
void setupCompass() {
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(LSM303::CRB_REG_M, 0x60); // +/-2.5 gauss
  compass.writeReg(LSM303::CRA_REG_M, 0x1C); // 220 Hz
  delay(1000);
  compassReady = true;
  compassCalibrated = false;
}

// 色センサ準備の後に実施し、約5秒間その場で回転して磁気値の範囲を校正する。
bool calibrateCompass() {
  int16_t minX = 32767, minY = 32767;
  int16_t maxX = -32768, maxY = -32768;
  float previousAngle = 0.0f;
  float accumulatedTurn = 0.0f;
  bool havePreviousAngle = false;
  // 校正中はメインループを離れて回転しながら100回サンプリングする。
  motors.setSpeeds(200, -200);
  for (uint8_t i = 0; i < 100; ++i) {
    compass.read();
    Serial1.print("ID=1,MAG_X=");
    Serial1.print(compass.m.x);
    Serial1.print(",MAG_Y=");
    Serial1.println(compass.m.y);
    minX = min(minX, compass.m.x); maxX = max(maxX, compass.m.x);
    minY = min(minY, compass.m.y); maxY = max(maxY, compass.m.y);
    float angle = atan2((float)compass.m.y, (float)compass.m.x) * 180.0f / 3.14159265f;
    if (havePreviousAngle) {
      float delta = angle - previousAngle;
      while (delta > 180.0f) delta -= 360.0f;
      while (delta < -180.0f) delta += 360.0f;
      accumulatedTurn += delta;
    }
    previousAngle = angle;
    havePreviousAngle = true;
    delay(50);
  }
  motors.setSpeeds(0, 0);
  motorL = motorR = 0;

  const int32_t rangeX = (int32_t)maxX - minX;
  const int32_t rangeY = (int32_t)maxY - minY;
  Serial1.print("ID=1,CAL_X="); Serial1.print(rangeX);
  Serial1.print(",CAL_Y="); Serial1.println(rangeY);
  if (fabs(accumulatedTurn) > 30.0f)
    rotationHeadingSign = accumulatedTurn > 0.0f ? 1 : -1;
  if (rangeX < 40 || rangeY < 40) {
    compassCalibrated = false;
    return false;
  }
  magMinX = minX; magMaxX = maxX;
  magMinY = minY; magMaxY = maxY;
  compassCalibrated = true;
  return true;
}

// 磁気値を複数回読み取り、校正範囲を使って絶対方位を計算する。
static float readAbsoluteHeading() {
  int32_t x = 0, y = 0;
  for (uint8_t i = 0; i < 5; ++i) {
    compass.read();
    x += compass.m.x;
    y += compass.m.y;
  }
  float scaledX = (float)x / 5.0f;
  float scaledY = (float)y / 5.0f;
  if (compassCalibrated) {
    const float rangeX = (float)magMaxX - magMinX;
    const float rangeY = (float)magMaxY - magMinY;
    if (rangeX < 40.0f || rangeY < 40.0f) return -1.0f;
    scaledX = 2.0f * (scaledX - magMinX) / rangeX - 1.0f;
    scaledY = 2.0f * (scaledY - magMinY) / rangeY - 1.0f;
  }
  if (fabs(scaledX) < 0.001f && fabs(scaledY) < 0.001f) return -1.0f;
  float angle = atan2(scaledY, scaledX) * 180.0f / 3.14159265f;
  if (angle < 0.0f) angle += 360.0f;
  return angle;
}

// 起動時の向きを相対方位0度として記録する。
bool captureHeadingReference() {
  if (!compassReady) return false;
  const float angle = readAbsoluteHeading();
  if (angle < 0.0f) return false;
  headingReferenceDeg = angle;
  if (headingReferenceDeg < 0.0f) headingReferenceDeg += 360.0f;
  return true;
}

// 方位センサの初期化が完了しているか返す。
bool compassIsReady() { return compassReady; }

// 起動時に記録した向きを基準に、現在の方位を0～360度で返す。
float readHeadingDeg() {
  if (!compassReady) return -1.0f;
  const float absoluteAngle = readAbsoluteHeading();
  if (absoluteAngle < 0.0f) return -1.0f;
  float relativeAngle = absoluteAngle - headingReferenceDeg;
  while (relativeAngle < 0.0f) relativeAngle += 360.0f;
  while (relativeAngle >= 360.0f) relativeAngle -= 360.0f;
  return relativeAngle;
}

// 仮の共通方位合わせ。山登り・カップ回収の動作自体は担当メンバーが実装予定。
// 目標方位への最短旋回を行い、許容角内を2回連続で確認したら完了する。
bool turnToHeading(float targetDeg) {
  static uint8_t settledSamples = 0;
  if (!compassReady) {
    stopMotors();
    return false;
  }

  const float current = readHeadingDeg();
  if (current < 0.0f) {
    stopMotors();
    return false;
  }
  float error = targetDeg - current;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  if (fabs(error) <= HEADING_TOLERANCE_DEG) {
    stopMotors();
    if (++settledSamples >= 2) {
      settledSamples = 0;
      return true;
    }
    return false;
  }
  settledSamples = 0;
  const int speed = constrain((int)(fabs(error) * 1.2f), 35, ROTATE_SPEED);
  if (error * rotationHeadingSign > 0) rotateInPlace(speed);
  else drive(-speed, speed);
  return false;
}

// 山登り用
// 加速度センサーのX軸の値を取得する。
int readAccelX() {
  compass.readAcc(); //加速度センサーだけを読み取る
  return compass.a.x; //x軸の測定値を返す
}
// 左右方向の加速度を取得する
int readAccelY() {
  compass.readAcc();
  return compass.a.y;
}
// 加速度センサーのX軸・Y軸・Z軸を読み取り、診断情報を送信する。
void sendAccelDebug() {
  compass.readAcc();

  Serial1.print("ID=1,ACC_X=");
  Serial1.print(compass.a.x);

  Serial1.print(",ACC_Y=");
  Serial1.print(compass.a.y);

  Serial1.print(",ACC_Z=");
  Serial1.print(compass.a.z);

  Serial1.print(",I2C_STATUS=");
  Serial1.print(compass.last_status);

  Serial1.print(",TIMEOUT=");
  Serial1.println(compass.timeoutOccurred() ? 1 : 0);
}
