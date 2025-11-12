/*
 * ファイル名: pico_wall_follower.cpp
 * 役割: Pi 4からシリアルで「偏差」を受け取り、壁に接近するためのPID制御を行う
 * (CugoSDK.h を使用するバージョン)
 */

// CugoSDKライブラリをインクルード
#include "CugoSDK.h" 

// --- 調整用パラメータ ---
const float BASE_SPEED = 20.0f; // 壁追従は低速で (例: 20 RPM)

// --- PIDゲインの調整値 ---
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
// ★ 壁追従用に必ず再調整してください。
// ★ (cugo_pid_cam_serch_line.c++ の値はライントレース用です)
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
const float Kp = 0.5f;  // Pゲイン (要調整)
const float Ki = 0.02f; // Iゲイン (要調整)
const float Kd = 0.1f;  // Dゲイン (要調整)

const float INTEGRAL_LIMIT = 200.0f; // 積分の上限 (要調整)

// --- PID制御用のグローバル変数 ---
float integral = 0.0f;        
float previous_error = 0.0f;  

void setup() {
  // CugoSDK.cpp 内の cugo_init() を呼び出す
  cugo_init(); // シリアル通信(Serial)やモータードライバ(Serial1)が初期化される
  Serial.println("Pico Wall Follower Ready.");
}

void loop() {
  // Pi 4 (PC) からシリアルデータが来ていれば処理を実行
  if (Serial.available() > 0) {
    // Pythonから送られてきた一行（'\n'まで）を読み込む
    String serial_data = Serial.readStringUntil('\n');
    serial_data.trim(); // 余分な空白を削除

    float error = 0.0f; // 現在のズレ(偏差)
    bool wall_detected = true; 

    // コマンドに応じて現在のズレ(error)を設定
    if (serial_data.startsWith("R ")) {
      serial_data.remove(0, 2); 
      error = serial_data.toFloat(); // ズレは正
    }
    else if (serial_data.startsWith("L ")) {
      serial_data.remove(0, 2);
      error = -serial_data.toFloat(); // ズレは負
    }
    else if (serial_data.startsWith("S")) {
      error = 0.0f;
    }
    else if (serial_data.startsWith("N")){
      wall_detected = false; // 壁を検出できなかった
    }
    else {
      return; // 不明なコマンドは無視
    }

    // --- PID制御計算 ---
    
    // [A] 壁を検出できなかった場合 (N)
    if (!wall_detected) {
      // その場でゆっくり回転して探す (右回転)
      cugo_rpm_direct_instructions(-BASE_SPEED, BASE_SPEED);
      
      // 次の制御に影響が出ないようにPID関連の変数をリセット
      integral = 0.0f;
      previous_error = 0.0f;
      Serial.println("Wall Not Found. Searching...");
      return; 
    } 
    
    // [B] 壁を検出できた場合 (R, L, S)
    
    // I項 (積分): 
    integral += error;
    if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
    if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
    if (error == 0.0f) integral = 0.0f; // ズレがほぼ無い(S)なら積分をリセット

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

    // CugoSDKの関数でモーターに指示
    cugo_rpm_direct_instructions(left_rpm, right_rpm);

    // デバッグ用にPi 4側に情報を送り返す
    Serial.print("Err:");
    Serial.print(error);
    Serial.print(" PID Out:");
    Serial.print(pid_output);
    Serial.print(" RPM L/R: ");
    Serial.print(left_rpm);
    Serial.print("/");
    Serial.println(right_rpm);
  }
}