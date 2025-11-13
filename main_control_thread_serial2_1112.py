# ファイル名: main_control_thread_serial2.py (アーキテクチャ変更版)

import serial 
import time
import threading 

# === [修正箇所 START] ===
# 統合された関数と、カメラ取得用の関数をインポート
from robot_vision_debug import camera_capture_thread, vision_processing_thread
# === [修正箇所 END] ===


### --- 設定 ---
STEERING_THRESHOLD = 20 
CAMERA_INDEX = 2 # 使用するカメラは1台だけ
MAIN_LOOP_WAIT_MS = 50 
MAIN_LOOP_WAIT_SEC = MAIN_LOOP_WAIT_MS / 1000.0 
SERIAL_PORT = '/dev/ttyS0' 
SERIAL_BAUDRATE = 115200


def main():
    
    # --- 1. スレッド間共有変数の初期化 ---
    shared_state = {
        'steering_value': 0.0,
        'steering_success': False, 
        'stop': False, 
        'gravity_value': 0.0,
        # === [修正箇所 START] ===
        'latest_frame': None,     # カメラ取得スレッドが書き込むフレーム
        'new_frame_flag': False,  # フレーム更新通知
        # === [修正箇所 END] ===
    }
    
    lock = threading.Lock()

    # --- 2. シリアルポートの準備 ---
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: シリアルポート ({SERIAL_PORT}) を開きました。")
    except serial.SerialException as e:
        print(f"[メイン] エラー: シリアルポート ({SERIAL_PORT}) を開けません。{e}")
        return 
            
    # --- 3. 画像処理スレッドを起動 ---
    # === [修正箇所 START] ===
    # 「カメラ取得スレッド」を起動
    t_camera = threading.Thread(target=camera_capture_thread, 
                                 args=(CAMERA_INDEX, shared_state, lock))
    
    # 「画像処理スレッド（統合版）」を起動
    t_vision = threading.Thread(target=vision_processing_thread,
                               args=(shared_state, lock))
    
    print("[メイン]: カメラ取得スレッドを起動します...")
    t_camera.start()
    
    print("[メイン]: 画像処理スレッド（操舵＋重心）を起動します...")
    t_vision.start()
    # === [修正箇所 END] ===
    
    print(f"[メイン]: 制御ループを開始します。 (Ctrl+Cで終了)")
    
    try:
        while True:
            # --- 4-1. 共有変数を安全に読み出す ---
            with lock:
                if shared_state['stop']:
                    print("[メイン]: スレッドからの停止要求を検出。ループを終了します。")
                    break
                    
                current_steering_diff = shared_state['steering_value']
                is_steering_success = shared_state['steering_success'] 
                current_gravity_diff = shared_state['gravity_value']
            
            # --- 4-2. 操舵コマンドを生成 ---
            # (このロジックは前回と同じ。完璧です)
            
            steering_command = "S" 
            active_steering_diff = 0.0
            mode_text = "" 
            
            if is_steering_success:
                # 優先1: 操舵スレッド(消失点)が成功した -> その値を使う
                active_steering_diff = current_steering_diff
                mode_text = "LINE_DETECT"
            else:
                # 優先2 (フォールバック): 操舵が失敗 -> 重心スレッドの値を使う
                # (vision_threadが 'gravity_value' にフォールバック結果を入れている)
                active_steering_diff = current_gravity_diff
                mode_text = "GRAVITY (Fallback)"

            if abs(active_steering_diff) > STEERING_THRESHOLD:
                if active_steering_diff > 0:
                    steering_command = f"R {active_steering_diff:.2f}" 
                else:
                    steering_command = f"L {abs(active_steering_diff):.2f}"
                                    
            # --- 4-3. コマンド決定とシリアル送信 ---
            final_command = steering_command 
            
            if ser:
                try:
                    command_to_send = f"{final_command}\n" 
                    ser.write(command_to_send.encode('utf-8'))
                except serial.SerialException as e:
                    print(f"[メイン] エラー: シリアル書き込み失敗。{e}")
                    ser.close()
                    ser = None 
            
            # --- 4-5. 状態の表示 (標準出力) ---
            print(f"\r状態: DRIVING, Mode: {mode_text:<18}, "
                  f"ズレ: {active_steering_diff:6.2f}, " 
                  f"コマンド: {final_command:<10}", end="")
            
            # --- 4-7. メインループの待機 ---
            time.sleep(MAIN_LOOP_WAIT_SEC)
            
    except KeyboardInterrupt:
        print("\n[メイン]: Ctrl+Cを検出。全スレッドを停止します。")
    finally:
        # --- 5. 終了処理 ---
        print("\n[メイン]: 終了処理中...")
        
        with lock:
            shared_state['stop'] = True
        
        # === [修正箇所 START] ===
        t_camera.join()
        t_vision.join()
        # === [修正箇所 END] ===
        
        print("[メイン]: 全スレッドが終了しました。")
        
        if ser and ser.is_open:
            ser.close() 
            print("[メイン]: シリアルポートを閉じました。")
        
        print("[メイン]: プログラムを終了します。")


if __name__ == '__main__':
    main()
