import cv2
import pyrealsense2 as rs
import time
import numpy as np
import threading

# ---------------------------------------------------------
# カメラ取得用スレッド関数 (計測機能付き)
# ---------------------------------------------------------
def realsense_capture_thread(shared_state, lock):
    # --- 設定・パラメータ ---
    # RealSense設定 (16:9)
    color_w, color_h = 640, 480  
    depth_w, depth_h = 640, 480
    
    # FPS設定: HW=60fps, SW=40fps
    HARDWARE_FPS = 60    
    TARGET_FPS = 40      
    MIN_INTERVAL = 1.0 / TARGET_FPS
    
    print(f"[カメラ取得スレッド]: RealSenseの起動を試みます... (HW:{HARDWARE_FPS}fps -> SW:{TARGET_FPS}fps)")
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # ストリーム設定
    config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, HARDWARE_FPS)
    config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, HARDWARE_FPS)
    
    # 計測用リスト（全履歴保持）
    #all_process_times = []
    
    try:
        profile = pipeline.start(config)
        
        # 深度スケールの取得
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        with lock:
            shared_state['depth_scale'] = depth_scale
            
        print("[カメラ取得スレッド]: RealSense 起動完了。計測を開始します。")
        
        last_update_time = 0
        frame_count = 0
        
        while True:
            # 停止フラグ確認
            with lock:
                if shared_state['stop']:
                    break
            
            try:
                # 1. フレーム待機 (ハードウェア待ち。計測対象外)
                frames = pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError:
                print("[カメラ] フレーム取得タイムアウト")
                continue
            
            # 2. FPS間引き処理 (ターゲットFPS制御)
            current_time = time.perf_counter()
            elapsed_time = current_time - last_update_time
            if elapsed_time < MIN_INTERVAL:
                sleep_time = MIN_INTERVAL - elapsed_time # 「目標時間 - 経過時間」が正しい待機時間
                if sleep_time > 0:
                    time.sleep(sleep_time)
            #if current_time - last_update_time < MIN_INTERVAL:
            #    time.sleep(current_time - last_update_time)
            last_update_time = current_time
            
            # ==========================================
            # ★ 計測開始 (純粋な処理時間)
            # ==========================================
            #proc_start = time.perf_counter()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # Numpy配列化
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 共有メモリ更新
            with lock:
                shared_state['prev_frame'] = shared_state['latest_frame']
                shared_state['latest_frame'] = color_image
                shared_state['latest_depth'] = depth_image
                shared_state['new_frame_flag'] = True

            # ==========================================
            # ★ 計測終了 & 履歴保存
            # ==========================================
            #proc_end = time.perf_counter()
            #proc_time_ms = (proc_end - proc_start) * 1000
            #all_process_times.append(proc_time_ms)
            
            frame_count += 1

            # --- 1秒ごとのFPSと平均処理時間の表示 ---
    except Exception as e:
        print(f"[カメラ取得スレッド] 重大エラー: {e}")
        
    finally:
        # 終了処理とレポート表示
        try:
            pipeline.stop()
        except:
            pass
        
        print("[カメラ取得スレッド]: 終了しました。")


# ===================================================================
# 補助関数: 深度画像から距離を計算 (WallDetectorのロジックを移植)
# ===================================================================
def calculate_distance_logic(depth_image, depth_scale, side):
    if depth_image is None:
        return None

    H, W = depth_image.shape
    # ROIの定義 (画面の上下100px、幅30px)
    roi_h = 100
    roi_w = 30
    roi_y1 = (H // 2) - (roi_h // 2)
    roi_y2 = roi_y1 + roi_h
    
    if side == 'right':
        roi_x1 = W - roi_w
        roi_x2 = W
    elif side == 'left':
        roi_x1 = 0
        roi_x2 = roi_w
    else:
        return 0.0

    # ROI抽出と計算
    if roi_y1 < 0 or roi_y2 > H or roi_x1 < 0 or roi_x2 > W:
        return 0.0 # 範囲外安全策

    depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
    non_zero_depth = depth_roi[depth_roi > 0] # 0 (測定不能) を除外

    if non_zero_depth.size > 0:
        return np.mean(non_zero_depth) * depth_scale
    else:
        return 0.0
    
    
# ===================================================================
# スレッド5: 壁接近制御スレッド (修正版)
# ===================================================================
def wall_control_thread(shared_state, lock, ser, wall_side,serial_lock):
    
    #壁接近用の定数の定義
    TARGET_DISTANCE = 0.80  # 目標とする壁との距離 以前のプログラムを確認
    CONTROL_FPS = 15
    CONTROL_INTERVAL = 1.0 / CONTROL_FPS
    ERROR_THRESHOLD = 0.1   
    
    # ターゲットの決定
    target_wall_side = None
    if wall_side == 'left': 
        target_wall_side = 'right'
    elif wall_side == 'right': 
        target_wall_side = 'left'
    else: 
        return
    
    print(f"[壁接近制御]: 開始。Target: {target_wall_side} (Water at: {wall_side})")
    
    # ★修正: ここでカメラを初期化しない！
    
    with lock:
        shared_state['stop_wall_control'] = False 
    
    all_process_times = []
    #last_update_time = 0
    frame_count = 0
    prev_time = time.perf_counter()
    
    while True:
        with lock:
            if shared_state['stop'] or shared_state['stop_wall_control']:
                break
            
            # ★修正: 共有メモリからDepth画像とScaleを取得
            depth_img = shared_state.get('latest_depth')
            scale = shared_state.get('depth_scale', 0.001)
        
        loop_start = time.time()
        
        if depth_img is None:
            print("[壁制御] Depthデータ待機中...")
            time.sleep(0.1)
            continue
        
        #計測開始
        proc_start = time.perf_counter()
        
        # 1. 距離計算 (関数呼び出し)
        current_distance = calculate_distance_logic(depth_img, scale, target_wall_side)
        
        # 2. コマンド生成　#送信コマンドの統一をする
        command = "" 
        if current_distance == 0.0:
            command = "S\n" 
            print(" [CONTROL] 壁検出不能(0.0m) -> 'S'")
        elif current_distance is None:
            command = "S\n" 
            print(" [CONTROL] 深度データなし -> 'S'")
        else:
            error = current_distance - TARGET_DISTANCE
            if abs(error) < ERROR_THRESHOLD:
                command = "S\n" 
                print(f" [CONTROL] OK ({current_distance:.2f}m) -> 'S'")
            elif error > 0: # 遠い
                # 右壁ターゲットで遠い(右に寄りたい) -> Right
                # 左壁ターゲットで遠い(左に寄りたい) -> Left
                direction = "Rw" if target_wall_side == 'right' else "Lw"
                command = f"{direction} {abs(error):.2f}\n"
                print(f" [CONTROL] 遠い ({current_distance:.2f}m) -> '{direction}'")
            else: # 近い (error < 0)
                # 右壁ターゲットで近い(左に避けたい) -> Left
                # 左壁ターゲットで近い(右に避けたい) -> Right
                direction = "Lw" if target_wall_side == 'right' else "Rw"
                command = f"{direction} {abs(error):.2f}\n"
                print(f" [CONTROL] 近い ({current_distance:.2f}m) -> '{direction}'")
        
        #計測終了
        proc_end = time.perf_counter()
        proc_time_ms = (proc_end - proc_start) * 1000
        all_process_times.append(proc_time_ms)
        frame_count += 1
        
        #command_sending(ser, command, serial_lock)
        print(f"\r [壁制御] {target_wall_side}壁追従: {current_distance:.2f}m, Cmd: {command.strip()}", end="")
        
        current_time_check = time.perf_counter()
        if current_time_check - prev_time >= 1.0:
            actual_fps = frame_count / (current_time_check - prev_time)
            
            # 直近100フレームの平均処理時間
            recent_avg = 0
            if all_process_times:
                subset = all_process_times[-100:]
                recent_avg = sum(subset) / len(subset)
            
            print(f"\n[壁制御] FPS: {actual_fps:.2f} | Avg Proc Time: {recent_avg:.2f}ms")
            
            frame_count = 0
            prev_time = current_time_check
        
        elapsed = time.time() - loop_start
        wait_time = CONTROL_INTERVAL - elapsed
        if wait_time > 0:
            time.sleep(wait_time)

    print("\n[壁制御]: 終了しました。")

# ---------------------------------------------------------
# メイン実行関数
# ---------------------------------------------------------
def main():
    # 共有データとロックの初期化
    lock = threading.Lock()
    shared_state = {
        'latest_frame': None,
        'prev_frame': None,
        'latest_depth': None,
        'depth_scale': 0,
        'new_frame_flag': False,
        'stop': False,  # スレッド停止用フラグ
        'stop_wall_control': False
    }
    
    print("=== Main Process Start ===")
    
    # スレッドの作成と開始
    t = threading.Thread(target=realsense_capture_thread, args=(shared_state, lock))
    t.start()
    
    ser = None  # シリアルポートオブジェクト (必要に応じて初期化)
    serial_lock = threading.Lock()  # シリアル通信ロック    
    t_wall = threading.Thread(target=wall_control_thread, args=(shared_state, lock, ser, 'right', serial_lock))
    t_wall.start()
    
    # 起動待ち (少し待機してカメラが安定するのを待つ)
    time.sleep(2.0)
    
    print("Press 'q' in the window to stop...")

    try:
        while True:
            # 最新フレームの取り出し
            frame_to_show = None
            
            with lock:
                # 新しいフレームが来ていれば取得
                if shared_state['new_frame_flag'] and shared_state['latest_frame'] is not None:
                    frame_to_show = shared_state['latest_frame'].copy()
                    shared_state['new_frame_flag'] = False # フラグを下ろす
            
            # フレームがあれば表示
            #if frame_to_show is not None:
            #    cv2.imshow("Main Thread View", frame_to_show)
            
            # キー入力待ち (1ms)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[Main] Stop signal received.")
                break
            
            # CPU使用率を下げたい場合は少しsleepを入れる
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Main] Keyboard Interrupt.")
    
    finally:
        # 終了処理
        print("[Main] Stopping thread...")
        with lock:
            shared_state['stop'] = True
            shared_state['stop_wall_control'] = True
        
        # スレッドが終了してレポートを出すのを待つ
        t.join()
        cv2.destroyAllWindows()
        print("[Main] Program finished.")

if __name__ == "__main__":
    main()