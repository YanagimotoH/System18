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


// 山登り開始ボタンの描画
void drawClimbButton(int id) {

  // ロボットごとの表示領域の左上の座標を求める
  float panelX = (id == 1) ? width / 2.0 : 0;
  float panelY = (id == 2) ? height / 2.0 : 0;

  // Zumoのタイトルの横にボタンを配置する
  climbButtonX[id] = panelX + 145;
  climbButtonY[id] = panelY + 10;

  // ボタンの中心座標を求める
  float cx = climbButtonX[id] + CLIMB_BUTTON_SIZE / 2;
  float cy = climbButtonY[id] + CLIMB_BUTTON_SIZE / 2;

  // 通信が有効で接続されている場合のみボタンを使用可能にする
  boolean available = enabled[id] && connected[id];

  // 丸いボタンの枠線を設定する
  stroke(255);
  strokeWeight(1.5);

  // 使用可能な場合は緑色、使用できない場合は灰色にする
  if (available) {
    fill(40, 150, 90);
  } else {
    fill(75);
  }

  // 丸いボタンを描画する
  ellipse(cx, cy, CLIMB_BUTTON_SIZE, CLIMB_BUTTON_SIZE);

  // ボタンの中に山のマークを描画する
  stroke(255);
  strokeWeight(2.5);
  noFill();

  line(cx - 12, cy + 8, cx - 2, cy - 7);
  line(cx - 2, cy - 7, cx + 3, cy + 1);
  line(cx + 3, cy + 1, cx + 8, cy - 5);
  line(cx + 8, cy - 5, cx + 13, cy + 8);
  line(cx - 12, cy + 8, cx + 13, cy + 8);

  // 線の太さを初期状態に戻す
  strokeWeight(1);
}


// マウス操作
void mousePressed() {

  // 各ロボットの山登りボタンがクリックされたか確認する
  for (int i = 0; i < 3; i++) {

    // ボタンの中心座標を求める
    float cx = climbButtonX[i] + CLIMB_BUTTON_SIZE / 2;
    float cy = climbButtonY[i] + CLIMB_BUTTON_SIZE / 2;

    // マウス位置とボタン中心の距離を求める
    float dx = mouseX - cx;
    float dy = mouseY - cy;

    // 円の内側をクリックした場合
    if (dx * dx + dy * dy <=
        sq(CLIMB_BUTTON_SIZE / 2)) {

      // 通信が有効で、接続されているロボットの場合
      if (enabled[i] && connected[i] && ports[i] != null) {

        // Arduinoに山登り開始を表す'M'を送信する
        ports[i].write('M');

        // コンソールに操作内容を表示する
        println("Zumo " + (i + 1) + ": 山登り開始");
      }

      // ボタンがクリックされたため、他のボタンは確認しない
      return;
    }
  }
}
