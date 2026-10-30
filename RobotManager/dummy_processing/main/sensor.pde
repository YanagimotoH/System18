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
    
    //ArduinoのIDの確認用
    //println("COM" + portNames[id] + " : " + line);

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
      }

      // DISTの場合は距離センサの値を更新する
      else if (key.equals("DIST")) {
        distanceCm[id] = int(value);
      }

      // STATUSの場合はロボットの開始・停止状態を更新する
      else if (key.equals("STATUS")) {

        // ArduinoからSTARTEDを受信した場合
        if (value.equals("STARTED")) {
          state[id] = 1;

        // ArduinoからSTOPPEDを受信した場合
        } else if (value.equals("STOPPED")) {
          state[id] = 0;
          robotState[id] = "STOP";
          modeCode[id] = -1;
        }
      }
    }

    // 正しいロボットからデータを受信したことを記録する
    received[id] = true;

    // 最後にデータを受信した時刻を記録する
    lastReceiveTime[id] = millis();
  }
}
