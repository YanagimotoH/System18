#include <Arduino.h>
#include "RobotConfig.h"

// 起動時の向きを基準に45度ずつ確認し、最後に探索の前進方向（初期は0度）へ向いて一周する。
// 最後の1方向(0.0f)は searchTravelHeadingDeg に置き換わる（ライン回避後は反対向きになる）。
static const float SEARCH_HEADINGS_DEG[8] = {
  45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f, 0.0f
};
static uint8_t searchHeadingIndex = 0;
static bool searchForwarding = false;
static bool searchAiming = false;          // 前進する向きへ旋回中
static float searchAimHeadingDeg = NORTH_HEADING_DEG; // 今回の前進方位
static unsigned long searchPhaseStartTime = 0;
static bool searchInitialized = false;
static float searchTravelHeadingDeg = NORTH_HEADING_DEG; // 探索で前進する向き

// ライン回避の状態
static bool avoidInitialized = false;
static unsigned long avoidStartTime = 0;
static unsigned long avoidTurnStartTime = 0;
static float avoidTurnTargetDeg = -1.0f;    // 反対向きへの旋回目標（-1は旋回しない）
static uint8_t avoidPhase = 0;              // 0:後退、1:180度旋回
static RobotState stateBeforeAvoid = SEARCH;

// 探索の前進方向±SEARCH_FORWARD_SPREAD_DEGの範囲でランダムな前進方位を決める。
static void pickSearchAim() {
  static bool seeded = false;
  if (!seeded) { randomSeed(micros()); seeded = true; }
  float h = searchTravelHeadingDeg +
            (float)random(-SEARCH_FORWARD_SPREAD_DEG, SEARCH_FORWARD_SPREAD_DEG + 1);
  while (h < 0.0f) h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;
  searchAimHeadingDeg = h;
}

// 探索の一周・前進時間とカップ検出を初期化する。最初はランダムな向きへ旋回してから前進する。
void resetSearchPattern() {
  searchHeadingIndex = 0;
  searchForwarding = false;
  searchTravelHeadingDeg = NORTH_HEADING_DEG;
  pickSearchAim();
  searchAiming = true;
  searchInitialized = true;
  avoidInitialized = false;
}

// 有効な距離が検出範囲内なら、1回の測定で対象候補とする。
bool detectTarget() {
  // ゴール直後は自陣色の領域を出るまでカップ候補を無視する。
  return !sensors.goalDetected &&
         sensors.distanceCm >= 0 &&
         sensors.distanceCm <= TARGET_DETECTION_RANGE_CM;
}

// 現在の超音波距離を返す。
int targetDistance() {
  return sensors.distanceCm;
}

// 360度回転して対象を確認 → 前方±30度のランダムな向きへ1秒前進 → また回転、を繰り返す。
void search() {
  if (!searchInitialized) resetSearchPattern();

  if (sensors.targetDetected) {
    stopMotors();
    state = APPROACH;
    return;
  }

  // 前進する向き（前方±30度のランダム）へ旋回する。
  if (searchAiming) {
    if (turnToHeading(searchAimHeadingDeg)) {
      searchAiming = false;
      searchForwarding = true;
      searchPhaseStartTime = millis();
    }
    return;
  }

  if (searchForwarding) {
    if (millis() - searchPhaseStartTime < 1000UL) {
      moveForward(FORWARD_SPEED);
      return;
    }
    stopMotors();
    searchForwarding = false;
    searchHeadingIndex = 0;
    return;
  }

  // 45度ずつ確認して360度回る。最後は探索の前進方向（ライン回避後は反対向き）。
  const float targetHeadingDeg = (searchHeadingIndex == 7)
                                     ? searchTravelHeadingDeg
                                     : SEARCH_HEADINGS_DEG[searchHeadingIndex];
  if (turnToHeading(targetHeadingDeg)) {
    ++searchHeadingIndex;
    if (searchHeadingIndex >= 8) {
      searchHeadingIndex = 0;
      pickSearchAim();
      searchAiming = true;
    }
  }
}

static uint8_t cupMissCount = 0;

void resetCupLost() { cupMissCount = 0; }

// 前方APPROACH_LIMIT_CM以内にカップがない状態が続いたらtrue。運搬・ゴール処理中に使う。
bool cupLost() {
  const int d = sensors.distanceCm;
  if (d >= 0 && d <= APPROACH_LIMIT_CM) cupMissCount = 0;
  else if (cupMissCount < 255) ++cupMissCount;
  return cupMissCount >= CUP_LOST_SAMPLES;
}

// 検出した方向へ直進し、接近距離に入ったら運搬へ移る。
void approachTarget() {
  const int distanceCm = sensors.distanceCm;

  // 対象を見失った状態がCUP_LOST_SAMPLES回続いたら探索へ戻る（単発の測定失敗では戻らない）。
  static uint8_t lostCount = 0;
  if (!sensors.targetDetected) {
    if (++lostCount >= CUP_LOST_SAMPLES) {
      lostCount = 0;
      stopMotors();
      state = SEARCH;
    }
    return;
  }
  lostCount = 0;

  if (distanceCm >= 0 && distanceCm <= APPROACH_LIMIT_CM) {
    stopMotors();
    sensors.targetDetected = false;
    resetCarryMotion();
    state = CARRY;
    return;
  }

  moveForward(100);
}

// 状態sで「ラインを踏んだ」とみなす床色かを返す。
// 黒（外周）と相手チーム色は、どの状態でも回避して離れる。
// 自チーム色は、運搬中(CARRY)はゴール、ゴール処理中(GOAL)は自陣なので回避しない。
// ゴール直後(goalDetected)は自陣のラインから出る途中なので、自チーム色も回避しない。
bool isLineContact(RobotState s) {
  const FloorColor color = sensors.floorColor;
  if (color == FLOOR_BLACK || isOpponentColor(color)) return true;
  return color == MY_TEAM_COLOR && !sensors.goalDetected &&
         (s == SEARCH || s == APPROACH);
}

// ライン接触の回避(AVOID)を開始する。回避後に戻る状態fromを記録する。
static unsigned long avoidEndTime = 0;   // 前回の回避を終えた時刻（0はまだ一度も終えていない）
static uint8_t avoidReentryCount = 0;    // 回避直後にまたラインを踏んだ回数

void beginAvoid(RobotState from) {
  // 回避直後に再びラインを踏む状態が続く場合は、探索へ抜けるようにする。
  if (avoidEndTime != 0 && millis() - avoidEndTime <= AVOID_REENTRY_MS) {
    if (++avoidReentryCount >= AVOID_REENTRY_MAX) {
      avoidReentryCount = 0;
      from = SEARCH;
    }
  } else {
    avoidReentryCount = 0;
  }
  stateBeforeAvoid = from;
  avoidStartTime = millis();
  avoidTurnTargetDeg = -1.0f;
  avoidPhase = 0;
  avoidInitialized = true;
  state = AVOID;
}

// 回避を終え、回避前の状態に応じた続きの処理へ戻る。
static void endAvoid() {
  stopMotors();
  avoidInitialized = false;
  avoidEndTime = millis();

  if (stateBeforeAvoid == CARRY) {
    // 運搬中: ライン脱出後にゴール方位へ向き直してから、再びゴールへ前進する。
    resetCarryMotion();
    state = CARRY;
  } else if (stateBeforeAvoid == GOAL) {
    // ゴール処理中: ライン脱出後、ゴール後の処理の続きから再開する。
    state = GOAL;
  } else {
    // 探索・接近中: 接近中だった対象は見失ったものとして探索へ戻る。
    // 同じラインへ戻らないよう、探索の前進方向を脱出した向き（反対向き）に変える。
    sensors.targetDetected = false;
    resetSearchPattern();
    if (avoidTurnTargetDeg >= 0.0f) {
      searchTravelHeadingDeg = avoidTurnTargetDeg;
      pickSearchAim();
    }
    state = SEARCH;
  }
}

// ラインを踏んだら、AVOID_BACK_MS 後退してから180度旋回する。
// 回避後は回避前の状態に応じて続きの動作を変える。
//   SEARCH/APPROACH: 探索の前進方向を旋回後の向き（反対向き）にして探索へ戻る
//   CARRY          : ゴール方位へ向き直して運搬を再開する
//   GOAL           : ゴール処理の続きを再開する
void avoidObstacle() {
  if (!avoidInitialized) beginAvoid(SEARCH); // 想定外の経路でAVOIDに入った場合の保険

  const unsigned long now = millis();

  // 例外処理: 回避が長引いた場合は探索へ抜ける。
  if (now - avoidStartTime >= AVOID_MAX_MS) {
    avoidEndTime = now;
    resumeSearch(); // 停止し、探索状態へ戻す（avoidInitializedも解除される）
    return;
  }

  // 0: 一定時間後退
  if (avoidPhase == 0) {
    if (now - avoidStartTime < AVOID_BACK_MS) {
      moveBackward(BACKWARD_SPEED);
      return;
    }
    stopMotors();
    // 方位が取れれば現在方位+180度を目標にする。取れない場合は旋回時間で180度回る。
    const float heading = readHeadingDeg();
    if (heading >= 0.0f) {
      avoidTurnTargetDeg = heading + 180.0f;
      if (avoidTurnTargetDeg >= 360.0f) avoidTurnTargetDeg -= 360.0f;
    } else {
      avoidTurnTargetDeg = -1.0f;
    }
    avoidTurnStartTime = now;
    avoidPhase = 1;
    return;
  }

  // 1: その場で180度旋回。終わらない場合は上限時間で打ち切る。
  bool turned;
  if (avoidTurnTargetDeg >= 0.0f) {
    turned = turnToHeading(avoidTurnTargetDeg) ||
             now - avoidTurnStartTime >= AVOID_TURN_TIMEOUT_MS;
  } else {
    rotateInPlace(ROTATE_SPEED);
    turned = now - avoidTurnStartTime >= ROTATE_180_TIME_MS;
  }
  if (turned) endAvoid();
}