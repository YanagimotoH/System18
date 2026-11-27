#include <Wire.h>
#include <LSM303.h>
#include <math.h>
#include "RobotConfig.h"
#include <ZumoMotors.h>

// 地磁気センサによる方位制御の基礎処理。
// 山登りと山上のカップ回収動作はここでは未実装。
// 現在はセンサ校正、方位取得、指定方位への旋回だけを提供する仮の基盤。（統合して欲しい個所）
// 校正が10秒以内に成功しなかった場合は、旋回時間から方位を推定する固定方位モードで同じ関数群を使う。
// 固定方位モードの旋回時間はロボット別の固定値（RobotConfig.hのFIXED_ROTATE_180_TIME_MS）。
extern ZumoMotors motors;
static LSM303 compass;
static bool compassReady = false;
static bool compassCalibrated = false;
static bool headingReferenceCaptured = false;
static float headingReferenceDeg = 0.0f;
static int16_t magMinX, magMinY, magMaxX, magMaxY;
static int rotationHeadingSign = 1;

// 固定方位モード（校正失敗時）の状態。
// 磁気センサを使わず、基準を記録した向きを北=0°として旋回時間から現在方位を推定する。
// ロボットの旋回はturnToHeading()だけが行うため、その旋回時間を積算すれば方位を追跡できる。
// 右回り（rotateInPlace）を方位増加方向とする（東=90°が北の右側）。
static bool fixedHeadingMode = false;
static float fixedHeadingDeg = 0.0f;       // 推定した現在方位（0～360度）
static bool fixedTurning = false;          // 前回の呼び出しで旋回指令を出したか
static int fixedTurnDir = 1;               // 前回の旋回方向（+1:右回り、-1:左回り）
static unsigned long fixedLastTurnMs = 0;  // 前回の旋回指令の時刻

static unsigned long lastCompassInitMs = 0;
static uint8_t accelFailures = 0;
static const char* accelError = "NOT_FOUND";

// 共有I2Cバスとセンサの設定だけを復旧し、走行開始時の基準方位と校正値は保持する。
void initCompassHardware() {
  lastCompassInitMs = millis();
  accelFailures = 0;
  Wire.begin();
  compass.setTimeout(30);
  compassReady = compass.init();
  if (!compassReady) {
    accelError = "NOT_FOUND";
    return;
  }

  compass.enableDefault(); // 加速度は全軸50 Hz。LSM303Dには専用の磁気レジスタ設定も含む。
  if (compass.last_status == 0 && compass.getDeviceType() != LSM303::device_D) {
    compass.writeMagReg(LSM303::CRB_REG_M, 0x60); // +/-2.5 gauss
    if (compass.last_status == 0) compass.writeMagReg(LSM303::CRA_REG_M, 0x1C); // 220 Hz
  }
  compassReady = compass.last_status == 0;
  if (!compassReady) accelError = "I2C";
  else delay(20); // 有効化後、最初の加速度変換を待つ
}

void setupCompass() {
  initCompassHardware();
  delay(1000);
  compassCalibrated = false;
  headingReferenceCaptured = false;
}

// 色センサ準備の後に実施し、回転中に磁気値の範囲を校正する。
bool calibrateCompass() {
  compassCalibrated = false;
  headingReferenceCaptured = false;
  if (!compassReady) return false;
  int16_t minX = 32767, minY = 32767;
  int16_t maxX = -32768, maxY = -32768;
  float previousAngle = 0.0f;
  float accumulatedTurn = 0.0f;
  bool havePreviousAngle = false;
  // 校正中はメインループを離れて回転しながらサンプリングする。
  // 最低COMPASS_CAL_MIN_SAMPLES回取り、XY各軸の変化幅がCOMPASS_CAL_MIN_RANGE以上になれば成功。
  // CALIBRATION_TIMEOUT_MS(10秒)以内に成功しなければ失敗とする。
  motors.setSpeeds(200, -200);
  uint16_t samples = 0;
  const unsigned long calStart = millis();
  while (millis() - calStart < CALIBRATION_TIMEOUT_MS) {
    compass.readMag();
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
    ++samples;
    delay(30);
    if (samples >= COMPASS_CAL_MIN_SAMPLES &&
        ((int32_t)maxX - minX) >= COMPASS_CAL_MIN_RANGE &&
        ((int32_t)maxY - minY) >= COMPASS_CAL_MIN_RANGE) {
      break;
    }
  }
  motors.setSpeeds(0, 0);
  motorL = motorR = 0;

  const int32_t rangeX = (int32_t)maxX - minX;
  const int32_t rangeY = (int32_t)maxY - minY;
  Serial1.print("ID=1,CAL_X="); Serial1.print(rangeX);
  Serial1.print(",CAL_Y="); Serial1.println(rangeY);
  // 10秒以内に十分な変化幅が得られなければ校正失敗。falseを返し、呼び出し側が
  // 固定方位モードへ切り替える。
  if (rangeX < COMPASS_CAL_MIN_RANGE || rangeY < COMPASS_CAL_MIN_RANGE) {
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
    compass.readMag(); // 方位計算では磁気だけを読む
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

// 校正失敗時に、磁気センサを使わず旋回時間で方位を推定する固定方位モードへ切り替える。
void enableFixedHeadingMode() {
  fixedHeadingMode = true;
  compassCalibrated = false;
  headingReferenceCaptured = false;
  rotationHeadingSign = 1; // 右回り(rotateInPlace)で方位が増える
  fixedHeadingDeg = NORTH_HEADING_DEG;
  fixedTurning = false;
}

// 固定方位モード（磁気センサ不使用）かを返す。
bool isFixedHeadingMode() { return fixedHeadingMode; }

// 起動時の向きを相対方位0度として記録する。
bool captureHeadingReference() {
  if (fixedHeadingMode) {
    // 固定方位モードでは、今ロボットが向いている方向を北(0°)とみなす。
    fixedHeadingDeg = NORTH_HEADING_DEG;
    fixedTurning = false;
    headingReferenceCaptured = true;
    return true;
  }
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
  // 固定方位モードでは旋回時間から推定した方位を返す。
  if (fixedHeadingMode) return headingReferenceCaptured ? fixedHeadingDeg : -1.0f;
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

// 固定方位モード用の旋回。旋回時間から推定した方位と目標の差で、旋回方向と完了を判定する。
static bool turnToHeadingFixed(float targetDeg, float toleranceDeg) {
  if (!headingReferenceCaptured) {
    // 基準方位（北）を記録する前は旋回しない
    stopMotors();
    return false;
  }

  const unsigned long now = millis();
  // 前回の旋回指令で回った分を推定方位へ反映する。
  // 指令の間隔が空きすぎた場合は別の動作を挟んだ（中断された）とみなして加算しない。
  if (fixedTurning && now - fixedLastTurnMs <= FIXED_HEADING_MAX_STEP_MS) {
    fixedHeadingDeg += fixedTurnDir * FIXED_HEADING_DEG_PER_MS * (float)(now - fixedLastTurnMs);
    while (fixedHeadingDeg < 0.0f) fixedHeadingDeg += 360.0f;
    while (fixedHeadingDeg >= 360.0f) fixedHeadingDeg -= 360.0f;
  }
  fixedLastTurnMs = now;

  float error = targetDeg - fixedHeadingDeg;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  // 推定方位にはセンサ誤差がないため、許容角は呼び出し側の値と標準値の小さい方にする。
  const float tolerance =
      toleranceDeg < HEADING_TOLERANCE_DEG ? toleranceDeg : HEADING_TOLERANCE_DEG;
  if (fabs(error) <= tolerance) {
    stopMotors();
    fixedTurning = false;
    return true;
  }

  fixedTurnDir = error > 0.0f ? 1 : -1;
  const int speed = ROTATE_SPEED;
  if (fixedTurnDir > 0) rotateInPlace(speed); // 右回りで方位が増える
  else drive(-speed, speed);
  fixedTurning = true;
  return false;
}

// 目標方位への最短旋回を固定速度で行い、指定した許容角内に入ったら完了する。
bool turnToHeading(float targetDeg, float toleranceDeg) {
  // 校正に失敗している場合は、旋回時間による固定方位モードで旋回する。
  if (fixedHeadingMode) return turnToHeadingFixed(targetDeg, toleranceDeg);

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

// 山登りと通信で同じ読み取り判定を使う。失敗時には前回の値を返さない。
bool readAccelXY(int &x, int &y) {
  if (!compassReady && millis() - lastCompassInitMs >= 1000UL) initCompassHardware();
  if (!compassReady) return false;

  compass.timeoutOccurred();
  compass.readAcc();
  const bool timedOut = compass.timeoutOccurred();
  if (compass.last_status != 0) accelError = "I2C";
  else if (timedOut) accelError = "TIMEOUT";
  else if (compass.a.x <= -32760 || compass.a.x >= 32760) accelError = "RANGE";
  else {
    x = compass.a.x;
    y = compass.a.y;
    accelFailures = 0;
    return true;
  }

  if (++accelFailures >= 3) compassReady = false;
  return false;
}

void sendAccelDebug() {
  int ax, ay;
  const bool valid = readAccelXY(ax, ay);
  Serial1.print("ID=");
  Serial1.print(ROBOT_ID);
  if (valid) {
    Serial1.print(",ACC_X=");
    Serial1.println(ax);
  } else {
    Serial1.print(",AX_ERR=");
    Serial1.println(accelError);
  }
}
