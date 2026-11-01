import processing.serial.*;

// シリアル通信・ロボット管理用の変数
// 最大3台のZumoと通信するためのSerialオブジェクト
Serial[] ports = new Serial[3];

// 各Zumoが接続されているシリアルポート名
String[] portNames = {"COM4", "COM6", "COM9"};

// trueのロボットのみシリアル通信を行う
boolean[] enabled = {false, false, true};

// 各COMポートに接続するArduinoのID
int[] robotIds = {2, 3, 1};

// 各ロボットの接続状態を管理する
boolean[] connected = new boolean[3];

// 各ロボットからデータを受信したかを管理する
boolean[] received = new boolean[3];

// ロボットの動作状態を管理する配列
// 0：STOP、1：RUN
int[] state = {0, 0, 0};

// 受信した動作モードの番号
// -1は受信なしを表す
int[] modeCode = {-1, -1, -1};

// 加速度センサの値
float[] ax = new float[3];
float[] ay = new float[3];
float[] az = new float[3];

// ロボットの姿勢を管理する配列
float[] pitch = new float[3];
float[] roll = new float[3];

// カラーセンサの表示色
// 初期状態は灰色に設定する
int[][] sensorColor = {
  {128, 128, 128},
  {128, 128, 128},
  {128, 128, 128}
};

// 各ロボットの現在の状態を文字列で管理する
String[] robotState = {"STOP", "STOP", "STOP"};

// 距離センサの値
// -1は距離データを受信していない状態を表す
int[] distanceCm = {-1, -1, -1};

// 受信時刻を記録する変数
// 最後にデータを受信した時刻をロボットごとに保存する
int[] lastReceiveTime = new int[3];

// 各ロボットが動作中かを管理する
boolean[] running = new boolean[3];

// データが一定時間届かなかった場合のタイムアウト時間
final int TIMEOUT_MS = 1500;

// 山登りボタンの位置とサイズ
float[] climbButtonX = new float[3];
float[] climbButtonY = new float[3];
final float CLIMB_BUTTON_SIZE = 42;
// 山登り担当ロボットを管理する
// -1：担当なし、0～2：Processingの配列番号
int hillRobot = -1;
// 山登りミッションが開始されたか
boolean hillStarted = false;
// 山登りが完了したか
boolean hillCompleted = false;


// 初期設定
void setup() {
  // ウィンドウのサイズを設定する
  size(1200, 800);

  // 画面に表示する文字のフォントを設定する
  textFont(createFont("SansSerif", 16));

  // 各ロボットのシリアル通信を初期化する
  for (int i = 0; i < 3; i++) {

    // 通信を有効にしたロボットのみ接続する
    if (enabled[i]) {
      try {
        // 指定したポートと通信速度9600bpsで接続する
        ports[i] = new Serial(this, portNames[i], 9600);

        // 接続時に残っている受信データを消去する
        ports[i].clear();

        // 接続が成功したことを記録する
        connected[i] = true;

      } catch (Exception e) {
        // 接続に失敗した場合は未接続として扱う
        connected[i] = false;
      }
    }
  }
}


// 画面の描画とデータの更新
void draw() {
  // 背景を黒色にする
  background(0);

  // 各ロボットの受信データを確認する
  for (int i = 0; i < 3; i++) {

    // ロボットごとのセンサデータを受信する
    readSensorData(i);

    // 一定時間データが届かなければSTOP扱いにする
    if (!received[i] ||
        millis() - lastReceiveTime[i] > TIMEOUT_MS) {
      running[i] = false;
    }
  }

  // 画面を4分割する区切り線を描画する
  stroke(80);
  strokeWeight(1);
  line(width / 2, 0, width / 2, height);
  line(0, height / 2, width, height / 2);

  // 左上・右上・左下に各Zumoの情報を表示する
  drawZumo(0, 0, 0);
  drawZumo(1, width / 2, 0);
  drawZumo(2, 0, height / 2);

  // 右下に全ロボットの通信情報を表示する
  drawInformation(width / 2, height / 2);

  // 各Zumoの山登り開始ボタンを描画する
  for (int i = 0; i < 3; i++) {
    drawClimbButton(i);
  }
}

// 山登り担当を選択するためのボタンを描画する関数
// idにはロボットの配列番号（0～2）が入る
void drawClimbButton(int id) {
  // ロボットの表示位置に合わせてパネルの左上座標を決める
  float panelX = (id == 1) ? width / 2.0 : 0;
  float panelY = (id == 2) ? height / 2.0 : 0;

  // 山登りボタンの左上座標を設定する
  climbButtonX[id] = panelX + 145;
  climbButtonY[id] = panelY + 10;

  // ボタンの中心座標を計算する
  float cx = climbButtonX[id] + CLIMB_BUTTON_SIZE / 2;
  float cy = climbButtonY[id] + CLIMB_BUTTON_SIZE / 2;

  // enabled[id] : Processing側でロボットを使用する設定か
  // connected[id] : Arduinoと通信できているか
  // 両方trueの場合だけ担当ロボットに設定できる
  boolean available = enabled[id] && connected[id];
  // 現在、このロボットが山登り担当に選ばれているか
  boolean selected = (hillRobot == id);
  stroke(255);       // ボタンの枠線を白色にする
  strokeWeight(1.5); // 枠線の太さ

  if (!available) {
    // ロボットが未接続、または使用対象外の場合は灰色
    fill(75);
  } else if (selected) {
    // 山登り担当に選ばれている場合は緑色
    fill(30, 220, 100);
  } else {
    // 通常探索担当の場合は暗い緑色
    fill(40, 100, 60);
  }
  // 円形のボタンを描画する
  ellipse(
    cx,
    cy,
    CLIMB_BUTTON_SIZE,
    CLIMB_BUTTON_SIZE
  );
  // ボタンの中に山の形を描く
  stroke(255);
  strokeWeight(2);
  noFill();
  // 左側の山
  line(cx - 12, cy + 7, cx - 3, cy - 7);
  // 山の頂上から右側
  line(cx - 3, cy - 7, cx + 6, cy + 7);
  // 右側の山
  line(cx + 1, cy + 7, cx + 9, cy - 3);
  line(cx + 9, cy - 3, cx + 14, cy + 7);
  // 選択状態を文字で表示する
  fill(255);       // 文字色を白色にする
  textSize(12);    // 文字サイズ
  if (selected) {
    // 山登り担当に選ばれている場合
    text("山登り担当", cx - 30, cy + 32);
  } else {
    // 山登り担当ではない場合
    text("通常探索", cx - 25, cy + 32);
  }
  // 他の表示に影響しないよう文字サイズを戻す
  textSize(16);
}

// マウスがクリックされたときに呼び出される関数
void mousePressed(){
  // hillStarted  : 山登りミッションが開始されたか
  // hillCompleted: 山登りミッションが完了したか
  //
  // 一度ミッションが始まったら担当を変更できない。
  // 完了した後も同じ試合中は変更できない。
  if (hillStarted || hillCompleted) {
    return;
  }
  // 3台のロボットのボタンを順番に確認する
  for (int i = 0; i < 3; i++) {
    // ボタンの中心座標を計算する
    float cx = climbButtonX[i] + CLIMB_BUTTON_SIZE / 2;
    float cy = climbButtonY[i] + CLIMB_BUTTON_SIZE / 2;

    // マウス位置とボタン中心との距離を計算する
    float dx = mouseX - cx;
    float dy = mouseY - cy;

    // 円の内部をクリックしたか判定する
    // 中心からの距離の2乗が半径の2乗以下ならボタン内
    if (dx * dx + dy * dy <= sq(CLIMB_BUTTON_SIZE / 2)) {
      // 使用対象外・未接続・通信ポートがない場合は
      // 山登り担当に設定できない
      if (!enabled[i] || !connected[i] || ports[i] == null) {
        return;
      }
      // 現在の担当ロボットを再クリックした場合
      if (hillRobot == i) {
        // Arduinoに担当解除の命令を送信する
        ports[i].write("HILL_OFF\n");

        // Processing側の担当情報も解除する
        hillRobot = -1;
        println( "Zumo " + robotIds[i] + ": 山登り担当解除" );
      } else {
        // すでに別のロボットが担当だった場合は
        // そのロボットに担当解除の命令を送る
        if (hillRobot != -1) {
          ports[hillRobot].write("HILL_OFF\n");
          println( "Zumo " + robotIds[hillRobot] + ": 山登り担当解除" );
        }
        // 新しいロボットを山登り担当に設定する
        hillRobot = i;
        // Arduinoにも担当設定の命令を送る
        ports[i].write("HILL_ON\n");
        println( "Zumo " + robotIds[i] + ": 山登り担当に設定" );
      }
      // ボタンを処理したので関数を終了する
      return;
    }
  }
}
