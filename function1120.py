#ライブラリのインポート
import cv2
import numpy as np
import math
import time
import pyrealsense2 as rs
import serial

#robot_vision_debug2からのパラメータ
RESIZE_WIDTH = 240
MIN_NOISE_AREA = 45 

# ===================================================================
# スレッド 1: RealSense統合取得スレッド (変更箇所)
# ===================================================================
#アライメントを削除してうまくいくか試す
def realsense_capture_thread(shared_state, lock):
    # --- RealSense設定 ---
    W, H = 640, 480
    HARDWARE_FPS = 30    # デバイス側は30fpsで安定させる
    TARGET_FPS = 20      # ソフトウェア側で20fpsとして処理する
    MIN_INTERVAL = 1.0 / TARGET_FPS
    
    print(f"[カメラ取得スレッド]: RealSenseの起動を試みます... (HW:{HARDWARE_FPS}fps -> SW:{TARGET_FPS}fps)")
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # ★重要: RGBとDepthの両方を有効化
    # BGR8はOpenCV用、Z16は深度計算用
    config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, HARDWARE_FPS)
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, HARDWARE_FPS)
    
    try:
        profile = pipeline.start(config)
        
        # 距離計算に必要なスケール情報を取得
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        # 画像位置合わせ用 (RGBの画角にDepthを合わせる)
        align = rs.align(rs.stream.color)
        
        # スケールを共有しておく
        with lock:
            shared_state['depth_scale'] = depth_scale
            
        print("[カメラ取得スレッド]: RealSense 起動完了。")
        
        last_update_time = 0
        
        while True:
            with lock:
                if shared_state['stop']:
                    break
            
            # 1. フレーム待機 (ここは30fpsで回る)
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                print("[カメラ] フレーム取得タイムアウト")
                continue
            
            # 2. FPS間引き処理
            current_time = time.time()
            if current_time - last_update_time < MIN_INTERVAL:
                # 指定時間経過していなければ、データ更新せずスキップ
                continue
            
            last_update_time = current_time
            
            # 3. アライメント処理 (少し重いので間引き後に行う)
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # 4. Numpy配列化
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 5. 共有メモリ更新
            with lock:
                # RGB画像 (操舵・水検知用)
                shared_state['prev_frame'] = shared_state['latest_frame']
                shared_state['latest_frame'] = color_image
                
                # Depth画像 (壁制御用)
                shared_state['latest_depth'] = depth_image
                
                # フラグ更新
                shared_state['new_frame_flag'] = True

    except Exception as e:
        print(f"[カメラ取得スレッド] 重大エラー: {e}")
    finally:
        try:
            pipeline.stop()
        except:
            pass
        print("[カメラ取得スレッド]: RealSenseを停止しました。")

    
# ===================================================================
# スレッド 2: 画像処理スレッド (操舵 + 重心)
# ===================================================================
def vision_processing_thread(shared_state, lock):
    # --- 操舵用パラメータ ---
    CANNY_THRESHOLD1 = 100
    CANNY_THRESHOLD2 = 150
    HOUGH_THRESHOLD = 35 
    HOUGH_MIN_LINE_LENGTH = 35
    HOUGH_MAX_LINE_GAP = 10
    CLIP_LIMIT = 15.0
    TILE_GRID_SIZE = (4, 4)
    
    TARGET_FPS = 3.0
    INTERVAL = 1.0 / TARGET_FPS
    
    while True:
        start_time = time.time()
        with lock:
            if shared_state['stop']:
                break
            # 水検知中(壁追従中)は処理を抑制
            if shared_state.get('water_detected',False):
                shared_state['new_frame_flag'] = False
                time.sleep(0.01)
                continue
            
            if not shared_state.get('new_frame_flag', False):
                continue
            
            # データをコピーしてロック解除
            shared_state['new_frame_flag'] = False
            frame = shared_state['latest_frame']
            
            if frame is None:
                continue
        
        try:
            # === 1. リサイズと前処理 ===
            orig_height, orig_width = frame.shape[:2]
            aspect_ratio = orig_height / orig_width
            resize_height = int(RESIZE_WIDTH * aspect_ratio)
            resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
            height, width = resized_frame.shape[:2]
            image_center_x = width / 2
            gray_frame= cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
            
            # === 2. 操舵（消失点）検出 ===
            gray_steering = gray_frame
            clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
            adjusted = clahe.apply(gray_steering)
            blurred_again = cv2.GaussianBlur(adjusted, (7, 7), 0)
            edges = cv2.Canny(blurred_again, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
            
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edges, connectivity=8)
            cleaned_edges = np.zeros_like(edges)
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] > MIN_NOISE_AREA:
                    cleaned_edges[labels == i] = 255
            
            lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi/180, threshold=HOUGH_THRESHOLD, minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)
            
            diagonal_lines = []
            vp_x = width // 2
            steering_success = False 
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    angle_deg = math.degrees(angle_rad)
                    abs_angle_deg = abs(angle_deg)

                    is_horizontal = (abs_angle_deg <= 10) or (abs_angle_deg >= 175)
                    is_vertical = (50 <= abs_angle_deg <= 130)
                    
                    if is_horizontal or is_vertical or x1 == x2:
                        continue
                    m = (y2 - y1) / (x2 - x1)
                    c = y1 - m * x1
                    diagonal_lines.append((m, c))
            
            intersection_points = []
            if len(diagonal_lines) >= 2:
                for i in range(len(diagonal_lines)):
                    for j in range(i + 1, len(diagonal_lines)):
                        m1, c1 = diagonal_lines[i]
                        m2, c2 = diagonal_lines[j]
                        if abs(m1 - m2) < 1e-5 or m1*m2 >0 : continue
                        x = (c2 - c1) / (m1 - m2)
                        y = m1 * x + c1
                        if -width < x < width * 2 and -height < y < height * 2:
                            intersection_points.append((x, y))
            if intersection_points:
                x_coords = [p[0] for p in intersection_points]
                vp_x = int(np.median(x_coords))
                steering_success = True 
            
            steering_difference = vp_x - image_center_x
            
            # === 3. フォールバック処理 (重心検出) ===
            if not steering_success:
                inverted_array = 255 - gray_frame
                total_weight = np.sum(inverted_array)
                center_x = width / 2 
                if total_weight > 0:
                    x_coords = np.arange(width)
                    center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

                gravity_difference = center_x - image_center_x
                
                with lock:
                    shared_state['steering_success'] = False
                    shared_state['steering_value'] = gravity_difference
            else:
                with lock:
                    shared_state['steering_success'] = True
                    shared_state['steering_value'] = steering_difference
                    
        except Exception as e:
            print(f"[画像処理スレッド] エラー: {e}")
            pass
        
        elapsed = time.time() - start_time
        wait_time = INTERVAL - elapsed
        if wait_time > 0:
            time.sleep(wait_time)
    print("[画像処理スレッド]: 終了しました。")
    
# ===================================================================
# スレッド3: オプティカルフローによる水の検出
# ===================================================================

def analyze_trajectory(track):
    TRAJECTORY_MIN_DY = 20.0 
    TRAJECTORY_DRIFT_RATIO = 0.5
    start_point = track[0]; end_point = track[-1]
    start_x, start_y = start_point; end_x, end_y = end_point
    dx = end_x - start_x; dy = end_y - start_y
    if dy < TRAJECTORY_MIN_DY: return False
    if dy == 0: return False
    drift_ratio = abs(dx) / dy
    if drift_ratio > TRAJECTORY_DRIFT_RATIO: return False
    return True

def optical_flow_water_detection(shared_state, lock):
    RESIZE_WIDTH = 360
    TARGET_FPS = 10.0
    INTERVAL = 1.0 / TARGET_FPS
    TRACK_MAX_LEN = 50
    MIN_TRACKS = 40
    RE_DETECT_INTERVAL = 10
    DETECTION_TTL = 15
    
    feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=10, blockSize=7)
    lk_params = dict(winSize=(10, 10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    width_2_9 = RESIZE_WIDTH * 2 // 9
    width_7_9 = RESIZE_WIDTH * 7 // 9
    
    active_tracks = []
    frame_idx = 0
    waterfall_memory = {}
    resize_height = 0
    old_gray = None
    
    while True:
        loop_start_time = time.time()
        
        with lock:
            if shared_state['stop']:
                break
            # 壁追従中も水を監視し続けるか、あるいは止めるかは要調整
            # ここでは動き続ける設定
            
            # 最新フレームの取得
            if shared_state['latest_frame'] is None:
                time.sleep(0.1)
                continue
            frame = shared_state['latest_frame'].copy() # 安全のためコピー

        try:
            # --- 1. リサイズ ---
            if resize_height == 0:
                orig_height, orig_width = frame.shape[:2]
                aspect_ratio = orig_height / orig_width
                resize_height = int(RESIZE_WIDTH * aspect_ratio)
                if resize_height == 0: continue

            latest_resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
            frame_gray = cv2.cvtColor(latest_resized_frame, cv2.COLOR_BGR2GRAY)
            
            #初回の場合は前フレームがないのでスキップ
            if old_gray is None:
                old_gray = frame_gray
                continue

            # --- 2. オプティカルフロー ---
            new_tracks = []
            if active_tracks:
                p0 = np.float32([tr[-1] for tr in active_tracks]).reshape(-1, 1, 2)
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                for i, (track, (new_point, status)) in enumerate(zip(active_tracks, zip(p1, st))):
                    if status[0] == 0: 
                        continue
                    new_point_tuple = tuple(new_point.ravel())
                    new_x = new_point_tuple[0]
                    
                    if width_2_9 < new_x < width_7_9: 
                        continue
                    track.append(new_point_tuple)
                    if len(track) > TRACK_MAX_LEN: 
                        track.pop(0)
                    new_tracks.append(track)
            active_tracks = new_tracks
                    
            if len(active_tracks) < MIN_TRACKS or frame_idx % RE_DETECT_INTERVAL == 0:
                detection_mask = np.zeros(frame_gray.shape, dtype=np.uint8)
                detection_mask[:, 0:width_2_9] = 255
                detection_mask[:, width_7_9:RESIZE_WIDTH] = 255
                new_points = cv2.goodFeaturesToTrack(frame_gray, mask=detection_mask, **feature_params)
                
                if new_points is not None:
                    for p in new_points: 
                        active_tracks.append([tuple(p.ravel())])
                    
            candidate_tracks = [] 
            for track in active_tracks:
                if len(track) < 2:
                    continue
                if analyze_trajectory(track):
                    candidate_tracks.append(track)
                    
            expired_cells = []
            for cell in waterfall_memory:
                waterfall_memory[cell] -= 1
                if waterfall_memory[cell] <= 0: 
                    expired_cells.append(cell)
            for cell in expired_cells: 
                del waterfall_memory[cell]
            for track in candidate_tracks:
                if not track: 
                    continue
                end_point = track[-1]
                key = (int(end_point[0]), int(end_point[1]))
                waterfall_memory[key] = DETECTION_TTL
                
            frame_idx += 1
            
            left_detections = 0
            right_detections = 0
                    
            if waterfall_memory:
                for (center_x, center_y), ttl in waterfall_memory.items():
                    if center_x <= width_2_9:
                        left_detections += 1
                    elif center_x >= width_7_9: 
                        right_detections += 1
                    
            with lock:
                if left_detections > 0:
                    shared_state['water_detected'] = True
                    shared_state['wall_side'] = 'left'
                elif right_detections > 0:
                    shared_state['water_detected'] = True
                    shared_state['wall_side'] = 'right'
                else:
                    shared_state['water_detected'] = False
                    shared_state['wall_side'] = None
            
            old_gray = frame_gray.copy()                            
        except Exception as e:
            # print(f"[オプティカルフロー] エラー: {e}")
            pass 
        
        elapsed = time.time() - loop_start_time
        wait_time = INTERVAL - elapsed
        if wait_time > 0:
            time.sleep(wait_time)
    print("[オプティカルフロー]: 終了しました。")

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
def wall_control_thread(shared_state, lock, ser, wall_side):
    
    #壁接近用の定数の定義
    TARGET_DISTANCE = 0.75  # 目標とする壁との距離 (0.75m)
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
            
        # 1. 距離計算 (関数呼び出し)
        current_distance = calculate_distance_logic(depth_img, scale, target_wall_side)
        
        # 2. コマンド生成
        command = "" 
        if current_distance == 0.0:
            command = "N\n" 
            print(" [CONTROL] 壁検出不能(0.0m) -> 'N'")
        else:
            error = current_distance - TARGET_DISTANCE
            if abs(error) < ERROR_THRESHOLD:
                command = "S\n" 
                print(f" [CONTROL] OK ({current_distance:.2f}m) -> 'S'")
            elif error > 0: # 遠い
                # 右壁ターゲットで遠い(右に寄りたい) -> Right
                # 左壁ターゲットで遠い(左に寄りたい) -> Left
                direction = "R" if target_wall_side == 'right' else "L"
                command = f"{direction} {abs(error):.2f}\n"
                print(f" [CONTROL] 遠い ({current_distance:.2f}m) -> '{direction}'")
            else: # 近い (error < 0)
                # 右壁ターゲットで近い(左に避けたい) -> Left
                # 左壁ターゲットで近い(右に避けたい) -> Right
                direction = "L" if target_wall_side == 'right' else "R"
                command = f"{direction} {abs(error):.2f}\n"
                print(f" [CONTROL] 近い ({current_distance:.2f}m) -> '{direction}'")
            
        if ser and ser.is_open:
            try:
                ser.write(command.encode('utf-8'))
            except serial.SerialException as e:
                print(f"[壁制御] Serial Error: {e}")
        
        print(f"\r [壁制御] {target_wall_side}壁追従: {current_distance:.2f}m, Cmd: {command.strip()}", end="")

        elapsed = time.time() - loop_start
        wait_time = CONTROL_INTERVAL - elapsed
        if wait_time > 0:
            time.sleep(wait_time)

    print("\n[壁制御]: 終了しました。")