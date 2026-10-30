// 各Zumoの情報を画面に表示する
// id：ロボット番号
// x、y：表示領域の左上の座標
void drawZumo(int id, float x, float y) {

  // 画面を4分割したときの各領域の幅と高さ
  float pw = width / 2.0;
  float ph = height / 2.0;

  // Zumoのタイトルを表示する
  fill(255);
  textAlign(LEFT, TOP);
  textSize(20);
  text("ZUMO " + (id + 1), x + 20, y + 15);

  // RUN / STOPの表示
  // 通信が有効で、データが正常に届いているか確認する
  boolean communicationOK =
    enabled[id] &&
    connected[id] &&
    received[id] &&
    millis() - lastReceiveTime[id] <= TIMEOUT_MS;

  // 通信が正常で、ロボットの状態がRUNの場合
  boolean showRun = communicationOK && state[id] == 1;

  // RUNの場合は緑色、STOPの場合は灰色にする
  if (showRun) {
    fill(0, 255, 100);
  } else {
    fill(100);
  }

  // RUN / STOPを表す丸を描画する
  noStroke();
  ellipse(x + 45, y + 100, 28, 28);

  // ロボットの動作状態を文字で表示する
  fill(255);
  textSize(16);
  text(showRun ? "RUN" : "STOP",
       x + 68, y + 91);

  // 現在の動作モードを表示する
  text("MODE: " + getModeName(id), x + 20, y + 135);

  // 接続・受信状態を表示する
  // データを受信している場合は緑色、受信していない場合は赤色にする
  fill(received[id] ? color(0, 255, 100) : color(255, 100, 100));
  textSize(13);
  text(received[id] ? "RECEIVING" : "NO DATA",
       x + 20, y + 165);

  // カラーセンサの表示
  fill(255);
  textAlign(RIGHT, TOP);
  textSize(14);
  text("Color", x + pw - 25, y + 25);

  // センサから取得した色を四角形で表示する
  fill(sensorColor[id][0],
       sensorColor[id][1],
       sensorColor[id][2]);
  stroke(255);
  rectMode(CORNER);
  rect(x + pw - 65, y + 48, 40, 25);

  // ロボットの車体と進行方向を表示する
  // pitchの値に応じて車体の向きを変える
  drawCar(x + pw / 2, y + ph / 2 + 20, pitch[id]);

  // ロボットの傾きの数値を表示する
  fill(255);
  textAlign(LEFT, BOTTOM);
  textSize(15);

  // 前後方向の傾き
  text("PITCH: " + nf(pitch[id], 1, 1) + " deg",
       x + 20, y + ph - 35);

  // 左右方向の傾き
  text("ROLL: " + nf(roll[id], 1, 1) + " deg",
       x + 20, y + ph - 12);
}


// ロボットの車体と進行方向の矢印を描画する
// x、y：車体の中心座標
// angle：車体の回転角度
void drawCar(float x, float y, float angle) {

  // 現在の座標や回転などの描画設定を保存する
  pushMatrix();

  // 車体の描画位置を指定する
  translate(x, y);

  // pitchの角度に合わせて車体を回転させる
  // radians()で度数法からラジアンに変換する
  rotate(radians(angle));

  // 車体を四角形で描画する
  rectMode(CENTER);
  fill(80);
  stroke(255);
  strokeWeight(2);
  rect(0, 0, 90, 38);

  // 車体の進行方向を表す矢印を描画する
  stroke(0, 255, 100);
  strokeWeight(3);
  line(45, 0, 100, 0);
  line(87, -10, 100, 0);
  line(87, 10, 100, 0);

  // 保存していた描画設定に戻す
  popMatrix();

  // 次の描画に影響しないように設定を戻す
  rectMode(CORNER);
  strokeWeight(1);
}


// ロボットの動作モードを取得する
// id：ロボット番号
String getModeName(int id) {
  return robotState[id];
}


// 全ロボットの通信情報を画面右下に表示する
void drawInformation(float x, float y) {

  // Informationのタイトルを表示する
  fill(255);
  textAlign(LEFT, TOP);
  textSize(20);
  text("Information", x + 20, y + 15);

  // 各ロボットの情報を表示する
  textSize(15);

  for (int i = 0; i < 3; i++) {

    // ロボットごとの表示位置を設定する
    float ty = y + 65 + i * 65;

    // ロボット番号とシリアルポート名を表示する
    fill(255);
    text("Zumo " + (i + 1) + "  " + portNames[i],
         x + 20, ty);

    // 通信状態に応じて表示内容と色を変更する
    if (!enabled[i]) {

      // 通信を無効にしている場合
      fill(130);
      text("OFF", x + 330, ty);

    } else if (!connected[i]) {

      // シリアルポートに接続できていない場合
      fill(255, 100, 100);
      text("DISCONNECTED", x + 330, ty);

    } else if (received[i] &&
           millis() - lastReceiveTime[i] <= TIMEOUT_MS) {

      // 一定時間内にデータを受信している場合
      fill(0, 255, 100);
      text("RECEIVING", x + 330, ty);

    } else {

      // データが届いていない場合
      fill(255, 100, 100);
      text("NO DATA", x + 330, ty);
    }
  }
}
