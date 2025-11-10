# ファイル名: main_control_single_camera.py
# (robot_vision_single_camera.py と同じフォルダに保存してください)

import serial 
import time
import threading 
import cv2 # ★★★ GUIを使うので必要 ★★★

# ↓↓↓ ファイル名変更を反映 ↓↓↓
from robot_vision_single_camera import steering_thread_func 
# ★★ 壁検出と重心検出はインポートしない ★★
# , wall_thread_func ,gravity_thread_func 
# ↑↑↑ ファイル名変更を反映 ↑↑↑

### --- ステートマシンの状態定義 ---
STATE_DRIVING = 0
STATE_STOPPED = 1

### --- 設定 ---
# 停止・クールダウン (壁検出を使わないため、現在は使われない)
STOP_DURATION_SEC = 3.0    
STOP_COOLDOWN_SEC = 10.0   

# ロボット制御
STEERING_THRESHOLD = 20 

#操舵モード (LINE_DETECT のみ使用)
STEERING_MODE = 'LINE_DETECT' 
# STEERING_MODE = 'GRAVITY' # ★★ 重心検出は使わない ★★

# カメラ設定
CAMERA_INDEX_STEERING = 0 # 操舵用カメラの番号
# CAMERA_INDEX_WALL = 1     # ★★ 壁検出は使わない ★★
# CAMERA_INDEX_GRAVITY = 0 # ★★ 重心検出は使わない ★★

# メインループの周期 (ミリ秒)
MAIN_LOOP_WAIT_MS = 50 # ★★ cv2.waitKey() 用 ★★★
# MAIN_LOOP_WAIT_SEC = MAIN_LOOP_WAIT_MS / 1000.0 # (time.sleep() は使わない)


# ★★★ シリアルポート設定 ★★★
SERIAL_PORT = '/dev/ttyS0' 
# SERIAL_PORT = '/dev/ttyACM0' 
SERIAL_BAUDRATE = 115200


def main():
    
    # --- 1. スレッド間共有変数の初期化 ---
    shared_state = {
        'steering_value': 0.0,
        # 'wall_detected': 0, # ★★ 使わない ★★
        'stop': False, 
        # 'gravity_value': 0.0, # ★★ 使わない ★★
        
        # --- GUI表示用にフレームを追加 ★★★
        'steering_frame': None, 
        # 'wall_frame_right': None, # ★★ 使わない ★★
        # 'wall_frame_left': None,  # ★★ 使わない ★★
        # 'gravity_frame': None # ★★ 使わない ★★
    }
    
    lock = threading.Lock()

    # --- 2. シリアルポートの準備 ---
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: シリアルポート ({SERIAL_PORT}) を開きました。")
    except serial.SerialException as e:
        print(f"[メイン] エラー: シリアルポート ({SERIAL_PORT}) を開けません。{e}")
        print("[メイン]: シリアル通信なしで続行します（デバッグモード）。")
    except Exception as e:
        print(f"[メイン] 予期せぬエラー (シリアル): {e}")
        return
            
    # --- 3. 画像処理スレッドを起動 ---
    t_steering = threading.Thread(target=steering_thread_func, 
                                 args=(CAMERA_INDEX_STEERING, shared_state, lock))
    
    # ★★ 壁検出スレッドは起動しない ★★
    # t_wall = threading.Thread(target=wall_thread_func, 
    #                          args=(CAMERA_INDEX_WALL, shared_state, lock))
    
    # ★★ 重心検出スレッドは起動しない ★★
    # t_gravity = threading.Thread(target=gravity_thread_func,
    #                            args=(CAMERA_INDEX_GRAVITY, shared_state, lock))
    
    print("[メイン]: 操舵スレッドを起動します...")
    t_steering.start()
    # t_wall.start() # ★★ 起動しない ★★
    # t_gravity.start() # ★★ 起動しない ★★
    
    # --- 4. メイン制御ループ (ステートマシン) ---
    current_state = STATE_DRIVING
    stop_timer_end_time = 0.0
    stop_cooldown_end_time = 0.0
    
    print(f"[メイン]: 制御ループを開始します。操舵モード: {STEERING_MODE} (ウィンドウ選択中に 'q' で終了)")
    
    # --- GUI表示用のフレーム変数 ★★★
    frame_steering = None
    # frame_wall_right = None # ★★ 使わない ★★
    # frame_wall_left = None  # ★★ 使わない ★★
    # frame_gravity = None # ★★ 使わない ★★

    try:
        while True:
            current_time = time.time()
            
            # --- 4-1. 共有変数を安全に読み出す ---
            with lock:
                if shared_state['stop']:
                    print("[メイン]: スレッドからの停止要求を検出。ループを終了します。")
                    break
                    
                current_steering_diff = shared_state['steering_value']
                # is_wall_detected = (shared_state['wall_detected'] == 1) # ★★ 使わない ★★
                
                # --- ★★★ 描画フレームを取得 ★★★ ---
                if shared_state['steering_frame'] is not None:
                    frame_steering = shared_state['steering_frame'].copy()
                
                # current_gravity_diff = shared_state['gravity_value'] # ★★ 使わない ★★
            
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
            
            # ★★ 重心モードのロジックは使わない ★★
            # elif STEERING_MODE == 'GRAVITY':
            #     ( ... 省略 ... )
                                    
            # --- 4-3. ステートマシンによるコマンド決定 ---
            final_command = steering_command 

            if current_state == STATE_DRIVING:
                # ★★ 壁検出による停止ロジックをコメントアウト ★★
                # if is_wall_detected and current_time > stop_cooldown_end_time:
                #     print(f"[メイン]: !!! 壁を検出！ {STOP_DURATION_SEC}秒間停止します。 !!!")
                #     current_state = STATE_STOPPED
                #     stop_timer_end_time = current_time + STOP_DURATION_SEC
                #     stop_cooldown_end_time = current_time + STOP_COOLDOWN_SEC
                #     final_command = "H" 
                # else:
                #     final_command = steering_command
                final_command = steering_command # 常に操舵コマンドを送信

            elif current_state == STATE_STOPPED:
                # (この状態に入ることはないが、ロジックは残しておく)
                if current_time > stop_timer_end_time:
                    print("[メイン]: --- 停止時間終了。運転を再開します。---")
                    current_state = STATE_DRIVING
                    final_command = steering_command
                else:
                    final_command = "H"
            
            # --- 4-4. シリアル通信 ---
            if ser:
                try:
                    command_to_send = f"{final_command}\n" 
                    ser.write(command_to_send.encode('utf-8'))
                except serial.SerialException as e:
                    print(f"[メイン] エラー: シリアル書き込み失敗。{e}")
                    ser.close()
                    ser = None
            
            # --- 4-5. 状態の表示 (標準出力) ---
            state_text = "DRIVING" if current_state == STATE_DRIVING else "STOPPED"
            mode_text = f"Mode: {STEERING_MODE}"
            # is_wall_detected を削除
            print(f"状態: {state_text}, {mode_text}, "
                  f"ズレ: {active_steering_diff:6.2f}, "
                  f"コマンド: {final_command}")
            
            # --- 4-6. 画像の表示 (★★★ 操舵カメラのみ有効化 ★★★) ---
            if STEERING_MODE == 'LINE_DETECT':
                if frame_steering is not None:
                    cv2.imshow('Steering Camera', frame_steering)
                # if cv2.getWindowProperty('Gravity Camera', cv2.WND_PROP_VISIBLE) >= 1: # ★★ 使わない ★★
                #     cv2.destroyWindow('Gravity Camera') # ★★ 使わない ★★
            
            # ★★ 重心モードの表示は使わない ★★
            # elif STEERING_MODE == 'GRAVITY':
            #    ( ... 省略 ... )
            
            # ★★ 壁カメラの表示は使わない ★★
            # if frame_wall_right is not None:
            #     cv2.imshow('Wall Right (Top)', frame_wall_right)
            # if frame_wall_left is not None:
            #     cv2.imshow('Wall Left (Bottom)', frame_wall_left)

            # --- 4-7. メインループの待機 (★★★ cv2.waitKey に変更 ★★★) ---
            # time.sleep(MAIN_LOOP_WAIT_SEC) # ← GUIなしの場合
            
            # GUIありの場合 (qキーで終了)
            key = cv2.waitKey(MAIN_LOOP_WAIT_MS) & 0xFF
            if key == ord('q'):
                print("\n[メイン]: 'q'キーを検出。全スレッドを停止します。")
                break # ループを抜ける

    except KeyboardInterrupt:
        print("\n[メイン]: Ctrl+Cを検出。全スレッドを停止します。")
    finally:
        # --- 5. 終了処理 ---
        print("[メイン]: 終了処理中...")
        
        with lock:
            shared_state['stop'] = True
        
        t_steering.join()
        # t_wall.join() # ★★ 使わない ★★
        
        #if t_gravity.is_alive(): # ★★ 使わない ★★
        #    t_gravity.join() # ★★ 使わない ★★
        
        print("[メイン]: 操舵スレッドが終了しました。")
        
        if ser and ser.is_open:
            ser.close()
            print("[メイン]: シリアルポートを閉じました。")
        
        cv2.destroyAllWindows() # ★★★ GUIウィンドウを閉じる ★★★
        print("[メイン]: プログラムを終了します。")


if __name__ == '__main__':
    main()