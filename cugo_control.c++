#include "CugoSDK.h"

// --- 調整用パラメータ ---
const float BASE_SPEED = 35.0f; 

// モーターの最大・最小RPM
const float MAX_RPM = CUGO_MAX_MOTOR_RPM;
const float MIN_RPM = -CUGO_MAX_MOTOR_RPM; 

// --- PIDゲイン ---
const float Kp = 0.6f; 
const float Ki = 0.05f;
const float Kd = 0.3f;

// --- PID制御の高度な設定 ---
// Iゲインの積分値が溜まりすぎるのを防ぐ上限値（アンチワインドアップ）
const float INTEGRAL_LIMIT = 300.0f;

// --- PID制御用のグローバル変数 ---
float integral = 0.0f;
float previous_error = 0.0f;

void setup() {
  cugo_switching_reset = false;
  cugo_init(); // PCとのシリアル通信開始
}

/**
 * @brief ロボットを停止させ、PID制御変数もリセットする関数
 */
void stop_robot() {
  cugo_rpm_direct_instructions(0.0f, 0.0f); // モーター停止
  // PID制御変数をリセット
  integral = 0.0f;
  previous_error = 0.0f;
}

void loop() {
  // リモコンモードと自律モードの切り替え
  if (cugo_runmode == CUGO_RC_MODE) {
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(100);
  } else if (cugo_runmode == CUGO_CMD_MODE) {
    
    // PCからシリアルデータが来ていれば処理を実行
    if (Serial.available() > 0) {
      String serial_data = Serial.readStringUntil('\n');
      serial_data.trim(); // 余分な空白を削除

      float error = 0.0f; 
      bool pid_run = false; // このループでPID制御(前進)を実行するか？

      // --- コマンド解析 ---
      if (serial_data.startsWith("R ")) {
        serial_data.remove(0, 2); 
        error = serial_data.toFloat(); // ズレは正
        pid_run = true;
      }
      else if (serial_data.startsWith("L ")) {
        serial_data.remove(0, 2);
        error = -serial_data.toFloat(); // ズレは負
        pid_run = true;
      }
      else if (serial_data.startsWith("S")) {
        // "S" は ズレ 0 (直進) を意味する
        error = 0.0f;
        pid_run = true;
        
        // ★変更点: error=0 なので積分値をリセット（またはゆっくり減衰）
        // これにより、過去のズレが直進に影響するのを防ぐ
        integral *= 0.5f; // 例: 積分値を半分に減衰
      }
      else if (serial_data.startsWith("H")) {
        // ★追加: "H" (Halt) は完全停止
        stop_robot();
        // (デバッグ表示)
        Serial.println("CMD: H (Halt) -> RPM L/R: 0.0/0.0");
        return; // PID計算は行わない
      }
      else if (serial_data.startsWith("N")){
        // "N" (Not Found) は線未検出 -> その場で回転
        // (Python側で N コマンドを実装した場合の処理)
        cugo_rpm_direct_instructions(-BASE_SPEED, BASE_SPEED); // 左回転
        cugo_wait(100); // 短時間回転
        
        // PID変数をリセット
        integral = 0.0f;
        previous_error = 0.0f;
        
        // (デバッグ表示)
        Serial.println("CMD: N (Not Found) -> Rotating");
        return; // PID計算は行わない
      }
      else {
        // 知らないコマンドは無視
        return;
      }

      // --- PID制御計算 (R, L, S の場合のみ実行) ---
      if (pid_run) {
        
        // I項 (積分): ズレを蓄積
        integral += error;
        
        // I項のアンチワインドアップ (溜まりすぎ防止)
        if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
        if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

        // D項 (微分): 今回のズレと前回のズレの差分
        float derivative = error - previous_error;

        // PID出力の計算 (P項 + I項 + D項)
        float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // 次のループのために今回のズレを保存
        previous_error = error;

        // --- モーターへの指示 ---
        // PID出力を左右のモーター速度に反映させる
        float left_rpm = BASE_SPEED + pid_output;
        float right_rpm = BASE_SPEED - pid_output;

        // ★追加: モーターの最大・最小リミット
        left_rpm = constrain(left_rpm, MIN_RPM, MAX_RPM);
        right_rpm = constrain(right_rpm, MIN_RPM, MAX_RPM);

        cugo_rpm_direct_instructions(left_rpm, right_rpm);

        // --- デバッグ表示 ---
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
}