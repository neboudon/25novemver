import cv2
import pyrealsense2 as rs
import time
import numpy as np
import threading

# ---------------------------------------------------------
# カメラ取得用スレッド関数 (前回のまま変更なし)
# ---------------------------------------------------------
def realsense_capture_thread(shared_state, lock):
    # --- 設定・パラメータ ---
    color_w, color_h = 640, 360  
    depth_w, depth_h = 640, 480
    HARDWARE_FPS = 60    
    TARGET_FPS = 40      
    MIN_INTERVAL = 1.0 / TARGET_FPS
    
    print(f"[Thread] Camera starting... (Target: {TARGET_FPS}fps)")
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, HARDWARE_FPS)
    config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, HARDWARE_FPS)
    
    try:
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        with lock:
            shared_state['depth_scale'] = depth_scale
            
        print("[Thread] Camera ready.")
        last_update_time = 0
        
        while True:
            with lock:
                if shared_state['stop']: break
            
            try:
                frames = pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError: continue
            
            current_time = time.perf_counter()
            if current_time - last_update_time < MIN_INTERVAL:
                time.sleep(MIN_INTERVAL - (current_time - last_update_time))
            last_update_time = time.perf_counter()
            
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame: continue
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            with lock:
                shared_state['latest_frame'] = color_image
                shared_state['latest_depth'] = depth_image
                shared_state['new_frame_flag'] = True

    except Exception as e:
        print(f"[Thread] Error: {e}")
    finally:
        try: pipeline.stop()
        except: pass
        print("[Thread] Stopped.")

# ---------------------------------------------------------
# グラフ描画関数 (上り段差用に表示テキスト等を微修正)
# ---------------------------------------------------------
def draw_analysis_graph(depth_profile_inv, height, step_y_start, step_y_end):
    graph_w, graph_h = 640, 480
    graph_img = np.zeros((graph_h, graph_w, 3), dtype=np.uint8) + 255 

    cv2.line(graph_img, (0, graph_h-1), (graph_w, graph_h-1), (0,0,0), 2)
    cv2.line(graph_img, (0, 0), (0, graph_h), (0,0,0), 2)

    cv2.putText(graph_img, "X: Image Y (Top -> Bottom)", (10, graph_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(graph_img, "Y: 1/Z (Up=Near, Down=Far)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    x_scale = graph_w / height 
    MAX_INV_Z = 3.0  
    y_scale = graph_h / MAX_INV_Z 

    points = []
    for y, val in enumerate(depth_profile_inv):
        if np.isnan(val) or np.isinf(val): continue
        gx = int(y * x_scale)
        gy = int(graph_h - (val * y_scale))
        gy = max(0, min(graph_h - 1, gy))
        points.append((gx, gy))

    if len(points) > 1:
        cv2.polylines(graph_img, [np.array(points)], False, (255, 0, 0), 2)

    # 障害物を検出した場合
    if step_y_start is not None:
        # 開始位置(下)から終了位置(上)までを塗りつぶし
        sx = int(step_y_end * x_scale)   # 画面上の上側（ループの先）
        ex = int(step_y_start * x_scale) # 画面上の下側（ループの手前）
        
        cv2.rectangle(graph_img, (sx, 0), (ex, graph_h), (0, 0, 255), 2)
        cv2.putText(graph_img, "OBSTACLE", (sx, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return graph_img

# ---------------------------------------------------------
# メイン実行関数
# ---------------------------------------------------------
def main():
    lock = threading.Lock()
    shared_state = {
        'latest_frame': None, 'latest_depth': None, 'depth_scale': 0.001,
        'new_frame_flag': False, 'stop': False
    }
    
    t = threading.Thread(target=realsense_capture_thread, args=(shared_state, lock))
    t.start()
    time.sleep(2.0)
    print("=== Detection Start (Mode: Upward Step/Obstacle) ===")
    
    frame_count = 0
    start_time = time.time()
    prev_time = start_time
    process_times = []
    all_process_times = []
    
    try:
        while True:
            color_img = None
            depth_img = None
            d_scale = 0.001
            
            with lock:
                if shared_state['new_frame_flag'] and shared_state['latest_frame'] is not None:
                    color_img = shared_state['latest_frame']
                    depth_img = shared_state['latest_depth']
                    d_scale = shared_state['depth_scale']
                    shared_state['new_frame_flag'] = False
            
            if color_img is None or depth_img is None:
                time.sleep(0.001)
                continue
            
            # --- アルゴリズム計測開始 ---
            proc_start = time.perf_counter()
            
            h, w = depth_img.shape
            strip_width = 60
            center_x = w // 2
            x_start = center_x - (strip_width // 2)
            x_end = center_x + (strip_width // 2)
            
            # 1. 中央短冊切り出し
            strip_data = depth_img[:, x_start:x_end]
            
            # 2. 1/z プロファイル作成
            row_means = np.mean(strip_data, axis=1) * d_scale
            with np.errstate(divide='ignore', invalid='ignore'):
                row_means[row_means < 0.1] = np.nan # 近すぎるノイズ除去
                inv_z_profile = 1.0 / row_means
                inv_z_profile = np.nan_to_num(inv_z_profile, nan=0.0, posinf=0.0, neginf=0.0)

            # 3. 平滑化
            kernel_size = 15
            kernel = np.ones(kernel_size) / kernel_size
            inv_z_smooth = np.convolve(inv_z_profile, kernel, mode='same')
            
            # -------------------------------------------------------
            # 4. 上り段差（障害物）検出ロジック ★ここを変更★
            # -------------------------------------------------------
            obstacle_detected_start = None # 障害物の始まり（足元側）
            obstacle_detected_end = None   # 障害物の終わり（頭側）
            
            # (A) 足元の勾配（Ground Slope）を学習する
            # 画像の一番下（手前）付近のデータを使って「現在の坂の角度」を基準にする
            base_slope_list = []
            check_base_start = h - 20
            check_base_end = h - 40
            
            for k in range(check_base_start, check_base_end, -1):
                val_k = inv_z_smooth[k]
                val_prev = inv_z_smooth[min(h-1, k+5)]
                slope = val_prev - val_k # 正の値になるはず（手前の方が値が大きいから）
                if val_k > 0.5: # データが有効な場合のみ
                    base_slope_list.append(slope)
            
            # 基準勾配 (デフォルトは0.05程度と仮定)
            ground_slope = np.mean(base_slope_list) if base_slope_list else 0.05
            
            # (B) 探索ループ
            scan_start = h - 45
            scan_end = h // 3
            
            obstacle_pixel_count = 0     # 壁らしきものが何ピクセル続いたか
            REQUIRED_HEIGHT_PIXELS = 15  # 何ピクセル続いたら「乗り越え不可」とみなすか（調整箇所）
            
            for y in range(scan_start, scan_end, -1):
                val = inv_z_smooth[y]
                prev_val = inv_z_smooth[min(h-1, y+5)]
                
                # 勾配計算
                current_diff = prev_val - val
                
                # ■判定ロジック■
                # 1. current_diff が ground_slope に比べて著しく小さい（0に近い） -> 「壁」
                # 2. current_diff が ground_slope と同じくらい -> 「床（坂道）」
                
                # "壁"判定の閾値: 地面の勾配の30%以下しか変化しないなら「壁」とみなす
                is_wall = (current_diff < ground_slope * 0.3) and (val > 0.5)
                
                if is_wall:
                    obstacle_pixel_count += 1
                else:
                    # 壁が途切れたとき、これまで蓄積した高さが十分にあれば「障害物確定」
                    if obstacle_pixel_count > REQUIRED_HEIGHT_PIXELS:
                        obstacle_detected_start = y + obstacle_pixel_count
                        obstacle_detected_end = y
                        break # 一番手前の障害物を見つけたら終了
                    
                    # 十分な高さがなければリセット（ただの小石やノイズ）
                    obstacle_pixel_count = 0
            
            # ループを抜けた後、画面端まで壁が続いていた場合の処理
            if obstacle_detected_start is None and obstacle_pixel_count > REQUIRED_HEIGHT_PIXELS:
                obstacle_detected_start = scan_end + obstacle_pixel_count
                obstacle_detected_end = scan_end

            # --- アルゴリズム計測終了 ---
            proc_end = time.perf_counter()
            proc_time_ms = (proc_end - proc_start) * 1000
            all_process_times.append(proc_time_ms)
            frame_count += 1
            
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                recent_avg = sum(all_process_times[-100:]) / len(all_process_times[-100:]) if all_process_times else 0
                print(f"[Main] FPS: {fps:.2f}, Recent Avg Proc Time: {recent_avg:.2f}ms")
                frame_count = 0
                prev_time = curr_time

            # --- 描画処理 ---
            # 緑枠（解析エリア）
            cv2.rectangle(color_img, (x_start, 0), (x_end, h), (0, 255, 0), 1)
            
            # ▼▼▼ 追加コード: 探索リミットライン(上から1/3) ▼▼▼
            limit_y = h // 3
            # 黄色い横線を引く
            cv2.line(color_img, (0, limit_y), (w, limit_y), (0, 255, 255), 1)
            cv2.line(color_img, (0, h-45), (w, h-45), (0, 255, 255), 1)
            # 文字を表示
            cv2.putText(color_img, "Scan Limit (1/3)", (10, limit_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            # ▲▲▲ 追加コードここまで ▲▲▲
            
            # 障害物検知時の表示
            if obstacle_detected_start is not None:
                # バウンディングボックス（赤枠）
                # start(下側) から end(上側) までを囲む
                cv2.rectangle(color_img, (x_start, obstacle_detected_end), (x_end, obstacle_detected_start), (0, 0, 255), 2)
                
                # 文字表示
                label = "OBSTACLE"
                cv2.putText(color_img, label, (x_start - 80, obstacle_detected_start), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # 塗りつぶし（半透明風）
                overlay = color_img.copy()
                cv2.rectangle(overlay, (x_start, obstacle_detected_end), (x_end, obstacle_detected_start), (0, 0, 255), -1)
                cv2.addWeighted(overlay, 0.3, color_img, 0.7, 0, color_img)

            # 情報表示
            info_text = f"Time: {proc_time_ms:.2f}ms | Slope: {ground_slope:.3f}"
            cv2.putText(color_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            graph_img = draw_analysis_graph(inv_z_smooth, h, obstacle_detected_start, obstacle_detected_end)
            
            cv2.imshow("Main View (Upward Step)", color_img)
            cv2.imshow("1/Z Graph", graph_img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        with lock: shared_state['stop'] = True
        t.join()
        cv2.destroyAllWindows()
        
        if all_process_times:
            print("\n" + "="*40)
            print(" FINAL BENCHMARK REPORT")
            print("="*40)
            print("--- Per Frame Processing Time ---")
            # 全データの表示ループ
            for i, t in enumerate(all_process_times):
                print(f"Frame {i+1:04d}: {t:.3f} ms")
            
            # 統計の計算
            total_avg = sum(all_process_times) / len(all_process_times)
            max_time = max(all_process_times)
            min_time = min(all_process_times)
            
            print("-" * 40)
            print(f"Total Frames Processed : {len(all_process_times)}")
            print(f"Average Processing Time: {total_avg:.3f} ms")
            print(f"Max Processing Time    : {max_time:.3f} ms")
            print(f"Min Processing Time    : {min_time:.3f} ms")
            print("="*40)
        else:
            print("\nNo frames processed.")

if __name__ == "__main__":
    main()