//赤(118,54,56)
//青(39,69,101)
//黒(29,40,39)
// 状態名
enum {
  INIT,
  FORWARD,
  PUSH,
  BACKWARD,
  ROTATE,
  ROTATE_90
};

void move()
{
  static unsigned long start_time = 0;
  static bool detected = false;

  static int forwardSpeed = 150;
  static int backwardSpeed = -150;
  static int turnSpeed = 200;

  // 黒を検知した時に回転する時間
  static unsigned long turn90Time = 750;

  switch (mode_G)
  {
    case INIT: // 初期状態
      mode_G = FORWARD;
      detected = false;
      break;

    case FORWARD: // 探索：直進
      motorL_G = forwardSpeed;
      motorR_G = forwardSpeed;

      // 赤または青を踏んだら押し出し
      if (identify_color(118, 54, 56) || identify_color(39, 69, 101))  //赤 || 青                   
      {
        if (detected == false)
        {
          detected = true;
          mode_G = PUSH;
          start_time = millis();
        }
      }
      // 黒を踏んだら90度回転へ
      else if (identify_color(29, 40, 39))
      {
        if (detected == false)
        {
          detected = true;
          mode_G = ROTATE_90;
          start_time = millis();
        }
      }
      else
      {
        detected = false;
      }
      break;

    case PUSH: // 0.35秒間押し出す
      motorL_G = forwardSpeed;
      motorR_G = forwardSpeed;

      if (millis() - start_time >= 350)
      {
        mode_G = BACKWARD;
        start_time = millis();
      }
      break;

    case BACKWARD: // 1秒間後退
      motorL_G = backwardSpeed;
      motorR_G = backwardSpeed;

      if (millis() - start_time >= 1000)
      {
        mode_G = ROTATE;
        start_time = millis();
      }
      break;

    case ROTATE: // 1.5秒間回転
      motorL_G = turnSpeed;
      motorR_G = -turnSpeed;

      if (millis() - start_time >= 3000)
      {
        mode_G = FORWARD;
        detected = false;
      }
      break;

    case ROTATE_90: // 黒を検知したら回転
      motorL_G = turnSpeed;
      motorR_G = -turnSpeed;

      if (millis() - start_time >= turn90Time)
      {
        mode_G = FORWARD;
        detected = false;
      }
      break;
  }
}

int identify_color(int red, int green, int blue)
{
  float d2;
  float d2_max = 30; // パラメーター（適宜調整）

  d2 = pow(red - red_G, 2) + 
       pow(green - green_G, 2) + 
       pow(blue - blue_G, 2);
       
  if ( d2 < d2_max * d2_max )
    return 1;
  else
    return 0;
}
