# ファイル名: main_control_single_camera.py
# (robot_vision_single_camera.py と同じフォルダに保存してください)

import serial 
import time
import threading 
import cv2 # ★★★ GUIを使うので必要 ★★★

# ↓↓↓ ファイル名変更を反映 ↓↓↓
# from robot_vision_single_camera import steering_thread_func # ★★ 使わない ★★
from robot_vision_single_camera_grav import gravity_thread_func # ★★★ 重心検出をインポート ★★★
# ↑↑↑ ファイル名変更を反映 ↑↑↑

### --- ステートマシンの状態定義 ---
STATE_DRIVING = 0
STATE_STOPPED = 1

### --- 設定 ---
# 停止・クールダウン (壁検出を使わないため、現在は使われない)
STOP_DURATION_SEC = 3.0    
STOP_COOLDOWN_SEC = 10.0   

# ロボット制御
STEERING_THRESHOLD = 15

#操舵モード (★★ 重心検出に変更 ★★)
# STEERING_MODE = 'LINE_DETECT' # ★★ 使わない ★★
STEERING_MODE = 'GRAVITY' # ★★★ 重心検出を使う ★★★

# カメラ設定
# CAMERA_INDEX_STEERING = 0 # ★★ 使わない ★★
# CAMERA_INDEX_WALL = 1     # ★★ 使わない ★★
CAMERA_INDEX_GRAVITY = 0 # ★★★ 重心検出用カメラの番号 ★★★

# メインループの周期 (ミリ秒)
MAIN_LOOP_WAIT_MS = 50 # ★★ cv2.waitKey() 用 ★★★
# MAIN_LOOP_WAIT_SEC = MAIN_LOOP_WAIT_MS / 1000.0 # (time.sleep() は使わない)


# ★★★ シリアルポート設定 ★★★
SERIAL_PORT = '/dev/ttyAMA0' 
# SERIAL_PORT = '/dev/ttyACM0' 
SERIAL_BAUDRATE = 115200


def main():
    
    # --- 1. スレッド間共有変数の初期化 ---
    shared_state = {
        # 'steering_value': 0.0, # ★★ 使わない ★★
        # 'wall_detected': 0, # ★★ 使わない ★★
        'stop': False, 
        'gravity_value': 0.0, # ★★★ 有効化 ★★★
        
        # --- GUI表示用にフレームを追加 ★★★
        # 'steering_frame': None, # ★★ 使わない ★★
        # 'wall_frame_right': None, # ★★ 使わない ★★
        # 'wall_frame_left': None,  # ★★ 使わない ★★
        #'gravity_frame': None # ★★★ 有効化 ★★★
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
    # ★★ 操舵スレッドは起動しない ★★
    # t_steering = threading.Thread(target=steering_thread_func, 
    #                              args=(CAMERA_INDEX_STEERING, shared_state, lock))
    
    # ★★ 壁検出スレッドは起動しない ★★
    # t_wall = threading.Thread(target=wall_thread_func, 
    #                          args=(CAMERA_INDEX_WALL, shared_state, lock))
    
    # ★★★ 重心検出スレッドを起動 ★★★
    t_gravity = threading.Thread(target=gravity_thread_func,
                               args=(CAMERA_INDEX_GRAVITY, shared_state, lock))
    
    # print("[メイン]: 操舵スレッドを起動します...")
    # t_steering.start() # ★★ 起動しない ★★
    # t_wall.start() # ★★ 起動しない ★★
    print("[メイン]: 重心検出スレッドを起動します...")
    t_gravity.start() # ★★★ 起動する ★★★
    
    # --- 4. メイン制御ループ (ステートマシン) ---
    current_state = STATE_DRIVING
    stop_timer_end_time = 0.0
    stop_cooldown_end_time = 0.0
    
    print(f"[メイン]: 制御ループを開始します。操舵モード: {STEERING_MODE} (ウィンドウ選択中に 'q' で終了)")
    
    # --- GUI表示用のフレーム変数 ★★★
    # frame_steering = None # ★★ 使わない ★★
    # frame_wall_right = None # ★★ 使わない ★★
    # frame_wall_left = None  # ★★ 使わない ★★
    #frame_gravity = None # ★★★ 有効化 ★★★

    try:
        while True:
            current_time = time.time()
            
            # --- 4-1. 共有変数を安全に読み出す ---
            with lock:
                if shared_state['stop']:
                    print("[メイン]: スレッドからの停止要求を検出。ループを終了します。")
                    break
                
                # ★★ 操舵データは読み込まない ★★
                # current_steering_diff = shared_state['steering_value']
                # if shared_state['steering_frame'] is not None:
                #     frame_steering = shared_state['steering_frame'].copy()
                
                # ★★★ 重心データを取得 ★★★
                current_gravity_diff = shared_state['gravity_value']
                #if shared_state['gravity_frame'] is not None:
                #    frame_gravity = shared_state['gravity_frame'].copy()
            
            # --- 4-2. 操舵コマンドを生成 ---
            steering_command = "S" 
            active_steering_diff = 0.0
            
            # ★★ 消失点モードのロジックは使わない ★★
            # if STEERING_MODE == 'LINE_DETECT':
            #     active_steering_diff = current_steering_diff
            #     if abs(current_steering_diff) > STEERING_THRESHOLD:
            #         if current_steering_diff > 0:
            #             steering_command = f"R {current_steering_diff:.2f}" 
            #         else:
            #             steering_command = f"L {abs(current_steering_diff):.2f}" 
            
            # ★★★ 重心モードのロジックを有効化 ★★★
            if STEERING_MODE == 'GRAVITY':
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
                # ★★ 壁検出による停止ロジックはコメントアウト ★★
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
            
            # --- 4-6. 画像の表示 (★★★ 重心カメラのみ有効化 ★★★) ---
            # ★★ 消失点モードの表示は使わない ★★
            # if STEERING_MODE == 'LINE_DETECT':
            #     if frame_steering is not None:
            #         cv2.imshow('Steering Camera', frame_steering)
            
            # ★★★ 重心モードの表示を有効化 ★★★
            #if STEERING_MODE == 'GRAVITY':
            #    if frame_gravity is not None:
            #        cv2.imshow('Gravity Camera', frame_gravity)
                # (不要なウィンドウを閉じる処理)
                #if cv2.getWindowProperty('Steering Camera', cv2.WND_PROP_VISIBLE) >= 1:
                #    cv2.destroyWindow('Steering Camera')
            
            # ★★ 壁カメラの表示は使わない ★★
            # if frame_wall_right is not None:
            #     cv2.imshow('Wall Right (Top)', frame_wall_right)
            # if frame_wall_left is not None:
            #     cv2.imshow('Wall Left (Bottom)', frame_wall_left)

            # --- 4-7. メインループの待機 (★★★ cv2.waitKey に変更 ★★★) ---
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
        
        # t_steering.join() # ★★ 使わない ★★
        # t_wall.join() # ★★ 使わない ★★
        
        # ★★★ 重心スレッドの終了を待つ ★★★
        if t_gravity.is_alive():
           t_gravity.join()
        
        print("[メイン]: 重心検出スレッドが終了しました。")
        
        if ser and ser.is_open:
            ser.close()
            print("[メイン]: シリアルポートを閉じました。")
        
        #cv2.destroyAllWindows() # ★★★ GUIウィンドウを閉じる ★★★
        print("[メイン]: プログラムを終了します。")


if __name__ == '__main__':
    main()