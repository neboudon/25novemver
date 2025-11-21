#ライブラリのインポート
import serial
import time
import threading

#関数のインポート (realsense_capture_thread に変更)
from function1117 import realsense_capture_thread, vision_processing_thread, optical_flow_water_detection, wall_control_thread

#定数の定義
STEERING_THRESHOLD = 2 
MAIN_LOOP_WAIT_MS = 50 
MAIN_LOOP_WAIT_SEC = MAIN_LOOP_WAIT_MS / 1000.0 
SERIAL_PORT = '/dev/ttyS0' 
SERIAL_BAUDRATE = 115200

#メイン処理
def main():
    #共有変数の初期化
    shared_state ={
        'latest_frame': None,    # RGB画像 (Numpy)
        'latest_depth': None,    # Depth画像 (Numpy) ★追加
        'depth_scale': 0.001,    # 深度スケール ★追加
        'prev_frame': None,      # 一つ前のRGB
        'new_frame_flag': False, 
        'steering_value': 0.0, 
        'steering_success': False, 
        'water_detected': False, 
        'wall_side': None, 
        'stop': False, 
        'stop_wall_control': False, 
        'mode': 'DRIVING' 
    }

    lock = threading.Lock()

    #シリアルポートの準備
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: シリアルポート ({SERIAL_PORT}) を開きました。")
    except serial.SerialException as e:
        print(f"[メイン] エラー: シリアルポート ({SERIAL_PORT}) を開けません。{e}")
        return

    # ★変更: RealSense統合スレッドの準備
    # camera_index は不要になったため引数から削除
    t_camera = threading.Thread(target=realsense_capture_thread, args=(shared_state, lock))

    #画像処理スレッドの開始
    t_vision = threading.Thread(target=vision_processing_thread, args=(shared_state, lock))
    
    #オプティカルフロースレッドの開始
    t_optical = threading.Thread(target=optical_flow_water_detection, args=(shared_state, lock))
    
    #壁追従スレッド (ここではまだ生成しない)
    t_wall_control = None
    
    print("[メイン]: RealSense統合スレッドを起動します...")
    t_camera.start()
    print("[メイン]: 画像処理スレッド（操舵＋重心）を起動します...")
    t_vision.start()
    print("[メイン]: オプティカルフロースレッドを起動します...")
    t_optical.start()

    try:
        while True:
            # --- 共有変数を安全に読み出す ---
            with lock:
                if shared_state['stop']:
                    print("[メイン]: スレッドからの停止要求を検出。ループを終了します。")
                    break
                
                current_mode = shared_state['mode'] 
                is_water_detected = shared_state['water_detected'] 
                detected_wall_side = shared_state['wall_side'] 
                
            # --- モード切替ロジック ---
            if is_water_detected and current_mode == 'DRIVING':
                print(f"\n[メイン] モード変更: DRIVING -> WALL_FOLLOWING (水検出: {detected_wall_side})")
                
                with lock:
                    shared_state['mode'] = 'WALL_FOLLOWING'
                    shared_state['stop_wall_control'] = False
                    
                # 壁制御スレッドを起動
                t_wall_control = threading.Thread(target=wall_control_thread, args=(shared_state, lock, ser, detected_wall_side))
                t_wall_control.start()
                
            elif current_mode == 'WALL_FOLLOWING' and not is_water_detected:
                print(f"\n[メイン] モード変更: WALL_FOLLOWING -> DRIVING (水消失)")
                with lock:
                    shared_state['mode'] = 'DRIVING'
                    if t_wall_control is not None:
                        shared_state['stop_wall_control'] = True 
                
                if t_wall_control is not None:
                    t_wall_control.join(timeout=2.0) 
                    t_wall_control = None
            
            # --- 走行ロジック (DRIVING) ---
            if current_mode == 'DRIVING':
                
                with lock:
                    current_steering_diff = shared_state['steering_value']
                    is_steering_success = shared_state['steering_success']
                
                if is_steering_success:  
                    active_steering_diff = current_steering_diff
                    mode_text = "LINE_DETECT"
                else:
                    active_steering_diff = current_steering_diff
                    mode_text = "Fallback (GRAVITY)"
                
                steering_command = "S"
                if abs(active_steering_diff) > STEERING_THRESHOLD:
                    if active_steering_diff > 0:
                        steering_command = f"R {active_steering_diff:.2f}" 
                    else:
                        steering_command = f"L {abs(active_steering_diff):.2f}"
                    
                final_command = steering_command

                if ser:
                    try:
                        command_to_send = f"{final_command}\n" 
                        ser.write(command_to_send.encode('utf-8'))
                    except serial.SerialException as e:
                        print(f"[メイン] エラー: シリアル書き込み失敗。{e}")
                        ser.close()
                        ser = None
                
                print(f"\r状態: DRIVING, Mode: {mode_text:<18}, "
                    f"ズレ: {active_steering_diff:6.2f}, " 
                    f"コマンド: {final_command:<10}", end="")
            
            # --- 走行ロジック (WALL_FOLLOWING) ---
            elif current_mode == 'WALL_FOLLOWING':
                if t_wall_control is not None and not t_wall_control.is_alive():
                    print("\n[メイン] 警告: 壁制御スレッドが停止。DRIVINGへ復帰。")
                    with lock:
                        shared_state['mode'] = 'DRIVING'
                        shared_state['water_detected'] = False 
                    t_wall_control = None
                    continue
                
                # 壁制御スレッド側でprintしているので、ここではシンプルに
                # print(f"\r状態: WALL_FOLLOWING ...", end="")
                pass
            
            time.sleep(MAIN_LOOP_WAIT_SEC)

    except KeyboardInterrupt:
        print("\n[メイン]: キーボード割り込み検出。終了します。")
    finally:
        print("\n[メイン]: 終了処理中...")
        
        with lock:
            shared_state['stop'] = True
            shared_state['stop_wall_control'] = True 
        
        if t_wall_control is not None and t_wall_control.is_alive():
            t_wall_control.join()
        t_optical.join()
        t_vision.join()
        t_camera.join()
        
        if ser and ser.is_open:
            ser.close() 
        
        print("[メイン]: プログラムを終了します。")

if __name__ == '__main__':
    main()