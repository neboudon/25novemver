#include "CugoSDK.h"

// --- 調整用パラメータ ---
// ロボットの基本となる前進速度 (RPM)
//const float BASE_SPEED = 80.0; 
const float BASE_SPEED = 35.0f; 

// モーターの最大・最小RPMを設定 (CugoSDK.hの定義を使用)
// これにより、計算結果がモーターの限界を超えるのを防ぎます。
const float MAX_RPM = CUGO_MAX_MOTOR_RPM;
const float MIN_RPM = -CUGO_MAX_MOTOR_RPM; // 後進も考慮

// --- PIDゲインの調整値 ---
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
// ★ ここが最も重要な調整箇所です。ロボットの挙動を見ながら変更してください。 ★
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

// Pゲイン: 現在のズレ(偏差)にどれだけ反応するかの係数。大きくすると反応が速くなるが、振動しやすくなる。
const float Kp = 0.6f; 
// Iゲイン: 過去のズレの蓄積にどれだけ反応するかの係数。定常的なズレをなくすが、大きくするとオーバーシュートしやすくなる。
const float Ki = 0.05f;
// Dゲイン: ズレの変化量(未来予測)にどれだけ反応するかの係数。振動を抑え、動きを滑らかにする。
const float Kd = 0.3f;

// --- PID制御の高度な設定 ---
// Iゲインの積分値が溜まりすぎるのを防ぐ上限値（アンチワインドアップ）
// これがないと、コースから外れたときに積分値が暴走し、復帰時に大きなオーバーシュートを起こします。
const float INTEGRAL_LIMIT = 300.0f;

// --- PID制御用のグローバル変数 ---
float integral = 0.0f;        // 積分項の蓄積値
float previous_error = 0.0f;  // 一つ前のループでのズレ(偏差)

void setup() {
  cugo_switching_reset = false;
  cugo_init(); // 初期化（この中でPCとのシリアル通信が開始されます）
}

void loop() {
  // リモコンモードと自律モードの切り替え
  if (cugo_runmode == CUGO_RC_MODE) {
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(100);
  } else if (cugo_runmode == CUGO_CMD_MODE) {
    int flag = 0; //線を検出できなかった場合のフラグ
    // PCからシリアルデータが来ていれば処理を実行
    if (Serial.available() > 0) {
      // Pythonから送られてきた一行（'\n'まで）を読み込む
      String serial_data = Serial.readStringUntil('\n');
      serial_data.trim(); // 余分な空白を削除

      float error = 0.0f; // 現在のズレ(偏差)を初期化
      bool line_detected = true; //線を検出したかどうかのフラグ

      // コマンドに応じて現在のズレ(error)を設定
      if (serial_data.startsWith("R ")) {
        serial_data.remove(0, 2); // "R 2"のR 2を削除する
        // 右に補正すべき時 = 目標(中心)は左にある = ズレは正
        error = serial_data.toFloat();
      }
      else if (serial_data.startsWith("L ")) {
        serial_data.remove(0, 2);
        // 左に補正すべき時 = 目標(中心)は右にある = ズレは負
        error = -serial_data.toFloat();
      }
      else if (serial_data.startsWith("S")) {
        // ズレが閾値以下 = 目標達成
        error = 0.0f;
      }
      else if (serial_data.startsWith("N")){
        //線を検出できなかったためロボットをその場で回転させる
        line_detected = false;
      }else{
        return;
      }

      // --- PID制御計算 ---
      // I項 (積分): ズレが閾値内の場合(Sコマンド)は積分値をリセット。それ以外は蓄積。
      if (!line_detected) {
        cugo_rpm_direct_instructions(-BASE_SPEED, BASE_SPEED);
        cugo_wait(100); // 短時間回転
      
        // 次の制御に影響が出ないようにPID関連の変数をリセット
        integral = 0.0f;
        previous_error = 0.0f;
        return; // この回のループはここで終了
    } 
      
      integral += error;
      

      if (flag == 1){
        //線を検出できなかった場合、ロボットをその場で回転させる
        cugo_rpm_direct_instructions(-BASE_SPEED, BASE_SPEED);
        cugo_wait(100);
        integral = 0; //積分値をリセット
        previous_error = 0; //前回の偏差もリセット
        return;
      }

      // D項 (微分): 今回のズレと前回のズレの差分
      float derivative = error - previous_error;

      // PID出力の計算 (P項 + I項 + D項)
      float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

      // 次のループのために今回のズレを保存
      previous_error = error;

      // --- モーターへの指示 ---
      // PID出力を左右のモーター速度に反映させる
      // 右に曲がりたい時(error > 0)、左を速く、右を遅くする
      float left_rpm = BASE_SPEED + pid_output;
      float right_rpm = BASE_SPEED - pid_output;

      cugo_rpm_direct_instructions(left_rpm, right_rpm);

      // デバッグ用にPC側に情報を表示
      Serial.print("Err:");
      Serial.print(error);
      Serial.print(" P,I,D Out:[");
      Serial.print(Kp * error);
      Serial.print(",");
      Serial.print(Ki * integral);
      Serial.print(",");
      Serial.print(Kd * derivative);
      Serial.print("] -> RPM L/R: ");
      Serial.print(left_rpm);
      Serial.print("/");
      Serial.println(right_rpm);
    }
  }
}