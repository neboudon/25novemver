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
    color_w, color_h = 640, 360  
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
    all_process_times = []
    
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
        prev_time = time.time()
        
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
            if current_time - last_update_time < MIN_INTERVAL:
                time.sleep(current_time - last_update_time)
            last_update_time = current_time
            
            # ==========================================
            # ★ 計測開始 (純粋な処理時間)
            # ==========================================
            proc_start = time.perf_counter()

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
            proc_end = time.perf_counter()
            proc_time_ms = (proc_end - proc_start) * 1000
            all_process_times.append(proc_time_ms)
            
            frame_count += 1

            # --- 1秒ごとのFPSと平均処理時間の表示 ---
            curr_time_check = time.perf_counter()
            if curr_time_check - prev_time >= 1.0:
                actual_fps = frame_count / (curr_time_check - prev_time)
                
                # 直近100フレームの平均処理時間
                recent_avg = 0
                if all_process_times:
                    subset = all_process_times[-100:]
                    recent_avg = sum(subset) / len(subset)
                
                print(f"[RealSense] FPS: {actual_fps:.2f} | Avg Proc Time: {recent_avg:.2f}ms")
                
                frame_count = 0
                prev_time = curr_time_check

    except Exception as e:
        print(f"[カメラ取得スレッド] 重大エラー: {e}")
        
    finally:
        # 終了処理とレポート表示
        try:
            pipeline.stop()
        except:
            pass
        
        if all_process_times:
            print("\n" + "="*40)
            print(" FINAL BENCHMARK REPORT")
            print("="*40)
            print("--- Per Frame Processing Time ---")
            
            # ※全データ表示 (行数が多い場合はコメントアウト推奨)
            # for i, t in enumerate(all_process_times):
            #     print(f"Frame {i+1:04d}: {t:.3f} ms")
            
            total_avg = sum(all_process_times) / len(all_process_times)
            max_time = max(all_process_times)
            min_time = min(all_process_times)
            
            print("-" * 40)
            print(f"Total Frames Processed : {len(all_process_times)}")
            print(f"Average Processing Time: {total_avg:.3f} ms")
            print(f"Max Processing Time    : {max_time:.3f} ms")
            print(f"Min Processing Time    : {min_time:.3f} ms")
            print("="*40 + "\n")
        else:
            print("\nNo frames processed.")
            
        print("[カメラ取得スレッド]: 終了しました。")


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
        'stop': False  # スレッド停止用フラグ
    }
    
    print("=== Main Process Start ===")
    
    # スレッドの作成と開始
    t = threading.Thread(target=realsense_capture_thread, args=(shared_state, lock))
    t.start()
    
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
            if frame_to_show is not None:
                cv2.imshow("Main Thread View", frame_to_show)
            
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
        
        # スレッドが終了してレポートを出すのを待つ
        t.join()
        cv2.destroyAllWindows()
        print("[Main] Program finished.")

if __name__ == "__main__":
    main()