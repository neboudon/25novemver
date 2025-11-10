# ファイル名: main_control_thread_serial.py
# (robot_vision_thread_headless.py と同じフォルダに保存してください)

import serial # ★★★ シリアル通信ライブラリをインポート ★★★
import time
import threading 
# import cv2 # ★★★ GUIを使わないので不要 ★★★

# ↓↓↓ ファイル名変更を推奨 ↓↓↓
# robot_vision_thread_headless.py からスレッド用の関数をインポート
from robot_vision_thread_headless1 import steering_thread_func, wall_thread_func ,gravity_thread_func
# ↑↑↑ ファイル名変更を推奨 ↑↑↑

### --- ステートマシンの状態定義 ---
STATE_DRIVING = 0
STATE_STOPPED = 1

### --- 設定 ---
# 停止・クールダウン
STOP_DURATION_SEC = 3.0    
STOP_COOLDOWN_SEC = 10.0   

# ロボット制御
STEERING_THRESHOLD = 20 

#操舵モード
STEERING_MODE = 'LINE_DETECT' 
#STEERING_MODE = 'GRAVITY' # 重心検出を使う場合はこちらを有効化

# カメラ設定
CAMERA_INDEX_STEERING = 0 # 操舵用カメラの番号
CAMERA_INDEX_WALL = 1     # 壁検出用カメラの番号
CAMERA_INDEX_GRAVITY = 0 # 重心検出用カメラの番号

# メインループの周期 (ミリ秒)
MAIN_LOOP_WAIT_MS = 50 # 50ms
MAIN_LOOP_WAIT_SEC = MAIN_LOOP_WAIT_MS / 1000.0 # ★★★ time.sleep() 用 ★★★


# ★★★ シリアルポート設定 ★★★
# ラズパイ4のGPIO (GP14, GP15) でPicoのUART (GP0, GP1など) と接続する場合
SERIAL_PORT = '/dev/ttyS0' 
# (注意: /dev/ttyS0 を使うには sudo raspi-config でシリアルコンソールを無効化する必要あり)

# ラズパイ4のUSBポートとPicoのUSBポートを接続する場合 (Pico側がCDCとして動作)
# SERIAL_PORT = '/dev/ttyACM0' 

SERIAL_BAUDRATE = 115200


def main():
    
    # --- 1. スレッド間共有変数の初期化 ---
    shared_state = {
        'steering_value': 0.0,
        'wall_detected': 0,
        'stop': False, 
        'gravity_value': 0.0,
        
        # --- GUI表示しないためフレームは削除 ---
        # 'steering_frame': None, 
        # 'wall_frame_right': None,
        # 'wall_frame_left': None,  
        # 'gravity_frame': None
    }
    
    lock = threading.Lock()

    # --- 2. シリアルポートの準備 (★★★ 追加 ★★★) ---
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: シリアルポート ({SERIAL_PORT}) を開きました。")
    except serial.SerialException as e:
        print(f"[メイン] エラー: シリアルポート ({SERIAL_PORT}) を開けません。{e}")
        print("[メイン]: シリアル通信なしで続行します（デバッグモード）。")
    except Exception as e:
        print(f"[メイン] 予期せぬエラー (シリアル): {e}")
        return # シリアル必須の場合はここで終了
            
    # --- 3. 画像処理スレッドを起動 ---
    t_steering = threading.Thread(target=steering_thread_func, 
                                 args=(CAMERA_INDEX_STEERING, shared_state, lock))
    
    #t_wall = threading.Thread(target=wall_thread_func, 
    #                         args=(CAMERA_INDEX_WALL, shared_state, lock))
    
    t_gravity = threading.Thread(target=gravity_thread_func,
                               args=(CAMERA_INDEX_GRAVITY, shared_state, lock))
    
    print("[メイン]: 操舵スレッドと壁検出スレッドを起動します...")
    t_steering.start()
    t_wall.start()
    
    # print("[メイン]: 重心検出スレッドを起動します...")
    # t_gravity.start()
    
    # --- 4. メイン制御ループ (ステートマシン) ---
    current_state = STATE_DRIVING
    stop_timer_end_time = 0.0
    stop_cooldown_end_time = 0.0
    
    print(f"[メイン]: 制御ループを開始します。操舵モード: {STEERING_MODE} (Ctrl+Cで終了)")
    
    # --- GUI表示しないためフレーム変数は不要 ---
    # frame_steering = None
    # frame_wall_right = None 
    # frame_wall_left = None 
    # frame_gravity = None

    try:
        while True:
            current_time = time.time()
            
            # --- 4-1. 共有変数を安全に読み出す ---
            with lock:
                if shared_state['stop']:
                    print("[メイン]: スレッドからの停止要求を検出。ループを終了します。")
                    break
                    
                current_steering_diff = shared_state['steering_value']
                is_wall_detected = (shared_state['wall_detected'] == 1)
                
                # --- フレーム取得処理は削除 ---
                
                current_gravity_diff = shared_state['gravity_value']
            
            # --- 4-2. 操舵コマンドを生成 ---
            steering_command = "S" 
            active_steering_diff = 0.0
            
            if STEERING_MODE == 'LINE_DETECT':
                active_steering_diff = current_steering_diff
                if abs(current_steering_diff) > STEERING_THRESHOLD:
                    if current_steering_diff > 0:
                        steering_command = f"R {current_steering_diff:.2f}" 
                    else:
                        steering_command = f"L {abs(current_steering_diff):.2f}" 
            
            elif STEERING_MODE == 'GRAVITY':
                active_steering_diff = current_gravity_diff
                # THRESHOLD は共通
                if abs(current_gravity_diff) > STEERING_THRESHOLD: 
                    if current_gravity_diff > 0:
                        steering_command = f"R {current_gravity_diff:.2f}" 
                    else:
                        steering_command = f"L {abs(current_gravity_diff):.2f}"
                                    
            # --- 4-3. ステートマシンによるコマンド決定 ---
            final_command = steering_command 

            if current_state == STATE_DRIVING:
                if is_wall_detected and current_time > stop_cooldown_end_time:
                    print(f"[メイン]: !!! 壁を検出！ {STOP_DURATION_SEC}秒間停止します。 !!!")
                    current_state = STATE_STOPPED
                    stop_timer_end_time = current_time + STOP_DURATION_SEC
                    stop_cooldown_end_time = current_time + STOP_COOLDOWN_SEC
                    final_command = "H" 
                else:
                    final_command = steering_command

            elif current_state == STATE_STOPPED:
                if current_time > stop_timer_end_time:
                    print("[メイン]: --- 停止時間終了。運転を再開します。---")
                    current_state = STATE_DRIVING
                    final_command = steering_command
                else:
                    final_command = "H"
            
            # --- 4-4. シリアル通信 (★★★ 追加 ★★★) ---
            if ser:
                try:
                    # Pico側で受信しやすいよう、コマンドの末尾に改行(\n)を追加
                    command_to_send = f"{final_command}\n" 
                    ser.write(command_to_send.encode('utf-8'))
                except serial.SerialException as e:
                    print(f"[メイン] エラー: シリアル書き込み失敗。{e}")
                    # ポートが切断された場合などを考慮
                    ser.close()
                    ser = None # エラーが続くのを防ぐ
            
            # --- 4-5. 状態の表示 (標準出力) ---
            state_text = "DRIVING" if current_state == STATE_DRIVING else "STOPPED"
            mode_text = f"Mode: {STEERING_MODE}"
            print(f"状態: {state_text}, {mode_text}, "
                  f"壁: {is_wall_detected}, "
                  f"ズレ: {active_steering_diff:6.2f}, "
                  f"コマンド: {final_command}")
            
            # --- 4-6. 画像の表示 (★★★ コメントアウト ★★★) ---
            # if STEERING_MODE == 'LINE_DETECT':
            #     if frame_steering is not None:
            #         cv2.imshow('Steering Camera', frame_steering)
            #     if cv2.getWindowProperty('Gravity Camera', cv2.WND_PROP_VISIBLE) >= 1:
            #         cv2.destroyWindow('Gravity Camera')
            
            # elif STEERING_MODE == 'GRAVITY':
            #     if frame_gravity is not None:
            #         cv2.imshow('Gravity Camera', frame_gravity)
            #     if cv2.getWindowProperty('Steering Camera', cv2.WND_PROP_VISIBLE) >= 1:
            #         cv2.destroyWindow('Steering Camera')
            
            # if frame_wall_right is not None:
            #     cv2.imshow('Wall Right (Top)', frame_wall_right)
            
            # if frame_wall_left is not None:
            #     cv2.imshow('Wall Left (Bottom)', frame_wall_left)

            # --- 4-7. メインループの待機 (★★★ time.sleep に変更 ★★★) ---
            time.sleep(MAIN_LOOP_WAIT_SEC)
            
            # key = cv2.waitKey(MAIN_LOOP_WAIT_MS) & 0xFF
            # if key == ord('q'):
            #     print("\n[メイン]: 'q'キーを検出。全スレッドを停止します。")
            #     break # ループを抜ける

    except KeyboardInterrupt:
        print("\n[メイン]: Ctrl+Cを検出。全スレッドを停止します。")
    finally:
        # --- 5. 終了処理 ---
        print("[メイン]: 終了処理中...")
        
        with lock:
            shared_state['stop'] = True
        
        t_steering.join()
        t_wall.join()
        
        #if t_gravity.is_alive():
        #    t_gravity.join()
        
        print("[メイン]: 全スレッドが終了しました。")
        
        if ser and ser.is_open:
            ser.close() # ★★★ シリアルポートを閉じる ★★★
            print("[メイン]: シリアルポートを閉じました。")
        
        # cv2.destroyAllWindows() # ★★★ GUIウィンドウを閉じる (不要) ★★★
        print("[メイン]: プログラムを終了します。")


if __name__ == '__main__':
    main()