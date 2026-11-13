// Arduinoから送信されたセンサデータを受信する
// id：Processing側で管理しているロボット番号
void readSensorData(int id) {

  // シリアルポートが存在しない場合は処理を終了する
  if (ports[id] == null) return;

  // 受信データが残っている間、繰り返し読み取る
  while (ports[id].available() > 0) {

    // 改行コードまでのデータを1行として読み取る
    String line = ports[id].readStringUntil('\n');

    // まだ改行までのデータが届いていない場合は次の処理へ進む
    if (line == null) continue;

    // 文字列の前後にある空白や改行を取り除く
    line = trim(line);

    // 空のデータの場合は処理しない
    if (line.length() == 0) continue;

    // カンマで区切り、各データに分割する
    String[] data = split(line, ',');

    // 受信したロボットのIDを保存する
    int receivedId = -1;

    // まずロボットIDを確認する
    for (int i = 0; i < data.length; i++) {

      // 各データを「項目名」と「値」に分割する
      String[] pair = split(data[i], '=');

      // 項目名がIDの場合、ロボット番号を取得する
      if (pair.length == 2 &&
          trim(pair[0]).equals("ID")) {

        receivedId = int(trim(pair[1]));
        break;
      }
    }

    // IDがないデータの場合はRGBだけ更新する
    if (receivedId == -1) {
      for (int i = 0; i < data.length; i++) {
        String[] pair = split(data[i], '=');
    
        if (pair.length != 2) continue;
    
        String key = trim(pair[0]);
        String value = trim(pair[1]);
    
        if (key.equals("R")) {
          sensorColor[id][0] = constrain((int)float(value), 0, 255);
        } else if (key.equals("G")) {
          sensorColor[id][1] = constrain((int)float(value), 0, 255);
        } else if (key.equals("B")) {
          sensorColor[id][2] = constrain((int)float(value), 0, 255);
        }
      }
    
      continue;
    }
    // ArduinoのID=1はProcessingの配列番号0に対応する
    // 自分が管理しているロボット以外のデータは無視する
    if (receivedId != robotIds[id]) continue;

    // 受信した各データを確認する
    for (int i = 0; i < data.length; i++) {

      // 各データを「項目名」と「値」に分割する
      String[] pair = split(data[i], '=');

      // 項目名と値の2つに分割できなかった場合は無視する
      if (pair.length != 2) continue;

      // 項目名と値をそれぞれ取得する
      String key = trim(pair[0]);
      String value = trim(pair[1]);

      // STATEの場合はロボットの動作状態を更新する
      if (key.equals("STATE")) {

        // 受信した状態を文字列として保存する
        robotState[id] = value;

        // STOP以外ならRUNとして表示する
        state[id] = value.equals("STOP") ? 0 : 1;

        // MODE欄に現在の動作状態を表示する
        if (value.equals("SEARCH")) {
          modeCode[id] = 0;
        } else if (value.equals("STOP")) {
          modeCode[id] = -1;
        } else {
          modeCode[id] = 2;
        }
      } else if (key.equals("DIST")) {  // DISTの場合は距離センサの値を更新する
        distanceCm[id] = int(value);
      } else if (key.equals("ACC_X")) { //X軸の加速度を受信
        ax[id] = float(value);
      
        // 平地でのX軸の値を基準に傾きを計算
        float flatAx = -50.0;
      
        // 加速度の値を角度に変換（暫定計算）
        pitch[id] = -degrees(
          asin(constrain((ax[id] - flatAx) / 16384.0, -1.0, 1.0))
        );
      } else if (key.equals("R")) { //RGB値
        sensorColor[id][0] = constrain((int)float(value), 0, 255);
      } else if (key.equals("G")) {
        sensorColor[id][1] = constrain((int)float(value), 0, 255);
      } else if (key.equals("B")) {
        sensorColor[id][2] = constrain((int)float(value), 0, 255);
      }else if (key.equals("STATUS")) { // STATUS：Arduinoから送られてきた状態通知を処理する
        // 通常の走行開始通知
        if (value.equals("STARTED")) {
          // ロボットが走行中であることを記録する
          state[id] = 1;
        } else if (value.equals("STOPPED")) { // 通常の走行停止通知
          // ロボットが停止したことを記録する
          state[id] = 0;
      
          // 表示用の状態をSTOPにする
          robotState[id] = "STOP";
      
          // 動作モードを未設定に戻す
          modeCode[id] = -1;
        } else if (value.equals("HILL_STARTED")) { // 山登り開始通知
          // 山登り担当のロボットからの通知か確認する
          if (hillRobot == id) {
            // 山登りミッションが開始されたことを記録する
            hillStarted = true;
            println( "Zumo " + robotIds[id] + ": 山登り開始！" );
          }
        } else if (value.equals("HILL_DONE")) { // 山登り完了通知
          // 山登り担当のロボットからの通知か確認する
          if (hillRobot == id) {
            // 山登りミッションが完了したことを記録する
            hillCompleted = true;

            // ミッション実行中の状態を解除する
            // hillCompletedはtrueのままなので担当変更はできない
            hillStarted = false;
            println( "Zumo " + robotIds[id] + ": 山登り完了！" );
          }
        }
      }
    }

    // 正しいロボットからデータを受信したことを記録する
    received[id] = true;

    // 最後にデータを受信した時刻を記録する
    lastReceiveTime[id] = millis();
  }
}
