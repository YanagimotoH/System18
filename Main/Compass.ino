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
static bool headingReferenceCaptured = false;
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
  headingReferenceCaptured = false;
}

// 色センサ準備の後に実施し、回転中に磁気値の範囲を校正する。
bool calibrateCompass() {
  compassCalibrated = false;
  headingReferenceCaptured = false;
  int16_t minX = 32767, minY = 32767;
  int16_t maxX = -32768, maxY = -32768;
  float previousAngle = 0.0f;
  float accumulatedTurn = 0.0f;
  bool havePreviousAngle = false;
  // 校正中はメインループを離れて回転しながら200回サンプリングする。
  motors.setSpeeds(200, -200);
  for (uint8_t i = 0; i < 200; ++i) {
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
    delay(30);
  }
  motors.setSpeeds(0, 0);
  motorL = motorR = 0;

  const int32_t rangeX = (int32_t)maxX - minX;
  const int32_t rangeY = (int32_t)maxY - minY;
  Serial1.print("ID=1,CAL_X="); Serial1.print(rangeX);
  Serial1.print(",CAL_Y="); Serial1.println(rangeY);
  // 旋回中に各軸が一度も変化しなければ校正できないため、再試行する。
  if (rangeX == 0 || rangeY == 0) {
    return false;
  }
  magMinX = minX; magMaxX = maxX;
  magMinY = minY; magMaxY = maxY;
  compassCalibrated = true;

  // 校正後の方位が実際の旋回で増えるか減るかを短く確認する。
  const float headingBeforeTurn = readAbsoluteHeading();
  motors.setSpeeds(200, -200);
  delay(250);
  motors.setSpeeds(0, 0);
  motorL = motorR = 0;
  const float headingAfterTurn = readAbsoluteHeading();
  float calibrationTurn = 0.0f;
  if (headingBeforeTurn >= 0.0f && headingAfterTurn >= 0.0f) {
    calibrationTurn = headingAfterTurn - headingBeforeTurn;
    while (calibrationTurn > 180.0f) calibrationTurn -= 360.0f;
    while (calibrationTurn < -180.0f) calibrationTurn += 360.0f;
  }
  if (calibrationTurn > 0.0f) rotationHeadingSign = 1;
  else if (calibrationTurn < 0.0f) rotationHeadingSign = -1;
  else rotationHeadingSign = accumulatedTurn < 0.0f ? -1 : 1;

  Serial1.print("ID=1,TURN_DEG="); Serial1.println(accumulatedTurn);
  Serial1.print("ID=1,CAL_DIRECTION_DEG="); Serial1.println(calibrationTurn);
  return true;
}

// 磁気値を複数回読み取り、校正範囲を使って絶対方位を計算する。
static float readAbsoluteHeading() {
  int32_t x = 0, y = 0;
  for (uint8_t i = 0; i < 5; ++i) {
    compass.read(); // 地磁気センサの値読み取り
    x += compass.m.x; // X軸の値を加算
    y += compass.m.y; // Y軸の値を加算
  }
  float scaledX = (float)x / 5.0f; // 平均値を計算
  float scaledY = (float)y / 5.0f; // 平均値を計算
  if (!compassCalibrated) return -1.0f;
  const float rangeX = (float)magMaxX - magMinX; // X軸の範囲を計算
  const float rangeY = (float)magMaxY - magMinY; // Y軸の範囲を計算
  if (rangeX == 0.0f || rangeY == 0.0f) return -1.0f;
  // 値を-1.0～1.0に正規化
  scaledX = 2.0f * (scaledX - magMinX) / rangeX - 1.0f;
  scaledY = 2.0f * (scaledY - magMinY) / rangeY - 1.0f;
  if (fabs(scaledX) < 0.001f && fabs(scaledY) < 0.001f) return -1.0f; // 値が小さすぎる場合は無効
  float angle = atan2(scaledY, scaledX) * 180.0f / 3.14159265f; // ラジアンを度に変換
  if (angle < 0.0f) angle += 360.0f; // 0～360度に変換
  return angle; // 絶対方位を返す
}

// 起動時の向きを相対方位0度として記録する。
bool captureHeadingReference() {
  if (!compassReady || !compassCalibrated) return false; // 校正前は基準方位を記録しない
  const float angle = readAbsoluteHeading(); // 絶対方位を読み取る
  if (angle < 0.0f) return false; // 読み取り失敗の場合は失敗
  headingReferenceDeg = angle; // 現在の方位を基準として記録
  if (headingReferenceDeg < 0.0f) headingReferenceDeg += 360.0f; // 0～360度に変換
  headingReferenceCaptured = true;
  return true;
}

// 方位センサの初期化が完了しているか返す。
bool compassIsReady() { return compassReady; }

// 起動時に記録した向きを基準に、現在の方位を0～360度で返す。
float readHeadingDeg() {
  if (!compassReady || !compassCalibrated || !headingReferenceCaptured) return -1.0f;
  const float absoluteAngle = readAbsoluteHeading(); // 絶対方位を読み取る
  if (absoluteAngle < 0.0f) return -1.0f; // 読み取り失敗の場合は無効
  float relativeAngle = absoluteAngle - headingReferenceDeg; // 基準方位との差を計算
  while (relativeAngle < 0.0f) relativeAngle += 360.0f; // 0～360度に変換
  while (relativeAngle >= 360.0f) relativeAngle -= 360.0f; // 0～360度に変換
  return relativeAngle;
}

int getHeadingRotationSign() {
  return rotationHeadingSign;
}

// 目標方位への最短旋回を固定速度で行い、指定した許容角内に入ったら完了する。
bool turnToHeading(float targetDeg, float toleranceDeg) {
  if (!compassReady) {
    // 方位センサが準備できていない場合はモーターを停止
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

  if (fabs(error) <= toleranceDeg) {
    stopMotors();
    return true;
  }
  const int speed = ROTATE_SPEED;
  if (error * rotationHeadingSign > 0) rotateInPlace(speed);
  else drive(-speed, speed);
  return false;
}

// 山登り用
// 加速度センサーのX軸とY軸を同時に読み取る
void readAccelXY(int &x, int &y) {
  compass.readAcc();

  x = compass.a.x;
  y = compass.a.y;
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
