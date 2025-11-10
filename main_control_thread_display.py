# ファイル名: main_control_thread_display.py
# (robot_vision_thread.py と同じフォルダに保存してください)

# import serial # シリアル通信はしない
import time
import threading 
import cv2
# robot_vision_thread.py からスレッド用の関数をインポート
from robot_vision_thread_display import steering_thread_func, wall_thread_func ,gravity_thread_func

### --- ステートマシンの状態定義 ---
STATE_DRIVING = 0
STATE_STOPPED = 1

### --- 設定 ---
# 停止・クールダウン
STOP_DURATION_SEC = 3.0    
STOP_COOLDOWN_SEC = 8.0   

# ロボット制御
STEERING_THRESHOLD = 20 

#操舵モード
STEERING_MODE = 'LINE_DETECT' 
#STEERING_MODE = 'GRAVITY' # 重心検出を使う場合はこちらを有効化

# カメラ設定
CAMERA_INDEX_STEERING = 0 # ★★★ 操舵用カメラの番号 ★★★
CAMERA_INDEX_WALL = 1     # ★★★ 壁検出用カメラの番号 ★★★
CAMERA_INDEX_GRAVITY = 0 # ★★★ 重心検出用カメラの番号 ★★★

# メインループの周期 (ミリ秒)
# time.sleep(0.2) の代わりに cv2.waitKey(50) を使う
MAIN_LOOP_WAIT_MS = 50 # 50ms (約20FPSでGUIを更新)


def main():
    
    # --- 1. スレッド間共有変数の初期化 ---
    shared_state = {
        'steering_value': 0.0,
        'wall_detected': 0,
        'stop': False, 
        'steering_frame': None, 
        'wall_frame_right': None, # ★★★ 変更 ★★★
        'wall_frame_left': None,   # ★★★ 変更 ★★★
        'gravity_value': 0.0,
        'gravity_frame': None
    }
    
    lock = threading.Lock()

    # --- 2. シリアルポートの準備 (削除) ---
            
    # --- 3. 画像処理スレッドを起動 ---
    t_steering = threading.Thread(target=steering_thread_func, 
                                 args=(CAMERA_INDEX_STEERING, shared_state, lock))
    
    t_wall = threading.Thread(target=wall_thread_func, 
                             args=(CAMERA_INDEX_WALL, shared_state, lock))
    
    t_gravity = threading.Thread(target=gravity_thread_func,
                               args=(CAMERA_INDEX_GRAVITY, shared_state, lock))
    
    print("[メイン]: 操舵スレッドまたは、重心検出スレッドと壁検出スレッドを起動します...")
    t_steering.start()
    t_wall.start()
    # t_gravity.start()
    
    
    # --- 4. メイン制御ループ (ステートマシン) ---
    current_state = STATE_DRIVING
    stop_timer_end_time = 0.0
    stop_cooldown_end_time = 0.0
    
    print("[メイン]: 制御ループを開始します。操舵モード: {STEERING_MODE}'q'キーで終了します。")
    
    # フレーム取得用の変数をループ外で初期化
    frame_steering = None
    frame_wall_right = None # ★★★ 変更 ★★★
    frame_wall_left = None  # ★★★ 変更 ★★★
    frame_gravity = None

    try:
        while True:
            current_time = time.time()
            
            # --- 4-1. 共有変数を安全に読み出す ---
            with lock:
                if shared_state['stop']:
                    print("[メイン]: スレッドからの停止要求を検出。ループを終了します。")
                    break
                    
                current_steering_diff = shared_state['steering_value']
                current_gravity_diff = shared_state['gravity_value']
                is_wall_detected = (shared_state['wall_detected'] == 1)
                

                if shared_state['steering_frame'] is not None:
                    frame_steering = shared_state['steering_frame'].copy()
                if shared_state['wall_frame_right'] is not None:
                    frame_wall_right = shared_state['wall_frame_right'].copy()
                if shared_state['wall_frame_left'] is not None:
                    frame_wall_left = shared_state['wall_frame_left'].copy()
                if shared_state['gravity_frame'] is not None:
                    frame_gravity = shared_state['gravity_frame'].copy()                
            
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
                    stop_timer_end_time = current_time + STOP_DURATION_SEC  # 「運転再開する時刻」をセット
                    stop_cooldown_end_time = current_time + STOP_COOLDOWN_SEC # 「次に停止できる時刻」をセット
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
            
            # --- 4-4. シリアル通信 (削除済み) ---
            
            # --- 4-5. 状態の表示 (標準出力) ---
            state_text = "DRIVING" if current_state == STATE_DRIVING else "STOPPED"
            mode_text = f"Mode: {STEERING_MODE}"
            print(f"状態: {state_text}, {mode_text}, "
                  f"壁: {is_wall_detected}, "
                  f"ズレ: {active_steering_diff:6.2f}, "
                  f"コマンド: {final_command}")
            
            # --- 4-6. 画像の表示 (★★★ 追加 ★★★) ---
            if STEERING_MODE == 'LINE_DETECT':
                if frame_steering is not None:
                    cv2.imshow('Steering Camera', frame_steering)
                try:
                    if cv2.getWindowProperty('Gravity Camera', cv2.WND_PROP_VISIBLE) >= 1:
                        cv2.destroyWindow('Gravity Camera')
                except cv2.error:
                    pass
            
            elif STEERING_MODE == 'GRAVITY':
                if frame_gravity is not None:
                    cv2.imshow('Gravity Camera', frame_gravity)
                try:
                    if cv2.getWindowProperty('Steering Camera', cv2.WND_PROP_VISIBLE) >= 1:
                        cv2.destroyWindow('Steering Camera')
                except cv2.error:
                    pass
            
            # ↓↓↓ ここから変更 ↓↓↓
            if frame_wall_right is not None:
                cv2.imshow('Wall Right (Top)', frame_wall_right)
            
            if frame_wall_left is not None:
                cv2.imshow('Wall Left (Bottom)', frame_wall_left)
            # ↑↑↑ ここまで変更 ↑↑↑

            # --- 4-7. メインループの待機 (★★★ 変更 ★★★) ---
            # time.sleep() の代わりに cv2.waitKey() を使用
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
        t_wall.join()
        #print("[メイン]: 両スレッドが終了しました。")
        
        #if t_gravity.is_alive():
        #    t_gravity.join()
        
        print("[メイン]: 全スレッドが終了しました。")
        
        cv2.destroyAllWindows() # ★★★ GUIウィンドウを閉じる ★★★
        print("[メイン]: プログラムを終了します。")


if __name__ == '__main__':
    main()