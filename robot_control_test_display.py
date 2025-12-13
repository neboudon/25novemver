import cv2
import pyrealsense2 as rs
import threading
import time
import numpy as np
import serial

#重心検出用のパラメータ
RESIZE_WIDTH = 240
OPTICAL_FLOW_RESIZE_WIDTH = 360

#シリアル通信用パラメータ
SERIAL_PORT = '/dev/ttyS0' 
SERIAL_BAUDRATE = 921600

#オプティカルフローに関するパラメータ
feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=10, blockSize=7)
lk_params = dict(winSize=(10, 10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

#軌跡追跡のパラメータ
TRACK_MAX_LEN = 50
RE_DETECT_INTERVAL = 10
MIN_TRACKS = 40
NEW_POINT_MIN_DIST = 15

#滝の検出パラメータ
TRAJECTORY_MIN_DY = 20.0 
TRAJECTORY_DRIFT_RATIO = 0.5
TRAJECTORY_MIN_POINTS = 5

#クラスタリングとメモリのパラメータ
CLUSTER_MIN_TRACKS = 3
CLUSTER_GRID_CELL_SIZE = 40
DETECTION_TTL = 15

#落下判定のパラメータ
FALL_MIN_DY = 10.0
MOVE_X_MAX = 10.0

#周囲判定のパラメータ
GROUP_RADIUS = 40.0
MIN_GROUP_SIZE = 2


# ==========================================
# 1. カメラ画像取得クラス (遅延ゼロの鍵)
# ==========================================
class CameraStream:
    def __init__(self):
        self.queue = rs.frame_queue(capacity=1, keep_frames=False)
        self.lock = threading.Lock()
        self.latest_frame = None
        self.capture_time = 0
        color_w, color_h = 640, 360  
        depth_w, depth_h = 640, 480
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, 60)
        profile = self.pipeline.start(self.config)
        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.capture_time = 0

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while True:
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            now = time.time()
            
            with self.lock:
                self.latest_color_frame = color_image
                self.latest_depth_frame = depth_image
                self.capture_time = now

    def get_latest(self):
        with self.lock:
            return self.latest_color_frame, self.latest_depth_frame, self.capture_time

# ==========================================
#オプティカルフローの補助関数
def judge_falling(track):
    start_point = track[0]; end_point = track[-1]
    start_x, start_y = start_point; end_x, end_y = end_point
    dy = end_y - start_y
    dx = end_x - start_x
    if dy < FALL_MIN_DY: return False
    if abs(dx) > MOVE_X_MAX: return False
    return True

def get_track_center(track):
    points = np.array(track)
    center = np.mean(points, axis=0)
    return center

def find_dense_falling_tracks(falling_candidates):
    if len(falling_candidates) <= MIN_GROUP_SIZE:
        return []
    centers = np.array([get_track_center(t) for t in falling_candidates])
    final_tracks = []
    for i in range(len(falling_candidates)):
        current_center = centers[i]
        distances = np.linalg.norm(centers - current_center, axis=1)
        neighbor_count = np.sum((distances < GROUP_RADIUS) & (distances > 0))
        if neighbor_count >= MIN_GROUP_SIZE:
            final_tracks.append(falling_candidates[i])
    return final_tracks


def process_optical_flow(color_img,optical_flow_resize_height,width_2_9,width_7_9,fast,waterfall_memory,old_gray,active_tracks,frame_idx):
    
    latest_resized_frame = cv2.resize(color_img, (OPTICAL_FLOW_RESIZE_WIDTH, optical_flow_resize_height), interpolation=cv2.INTER_AREA)
    frame_gray = cv2.cvtColor(latest_resized_frame, cv2.COLOR_BGR2GRAY)
    
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
    
    # --- 新規特徴点検出 ---
    if len(active_tracks) < MIN_TRACKS or frame_idx % RE_DETECT_INTERVAL == 0:
        detection_mask = np.zeros(frame_gray.shape, dtype=np.uint8)
        detection_mask[:, 0:width_2_9] = 255
        detection_mask[:, width_7_9:RESIZE_WIDTH] = 255
        
        keypoints = fast.detect(frame_gray, mask=detection_mask)
        if keypoints:
            keypoints = sorted(keypoints, key=lambda x: x.response, reverse=True)
            limit_count = feature_params['maxCorners'] 
            best_keypoints = keypoints[:limit_count]

            for kp in best_keypoints:
                x, y = kp.pt
                active_tracks.append([(x, y)])
        
    # --- 分析と描画 ---
    falling_candidates = []
    other_candidates = [] 
    for track in active_tracks:
        if len(track) < 2: 
            continue
        if judge_falling(track):
            falling_candidates.append(track)
        else:
            other_candidates.append(track)
    
    passed_tracks = find_dense_falling_tracks(falling_candidates)
    
    wall_side = "none" 
    
    if passed_tracks:
        left_count = 0
        right_count = 0
        center_x = OPTICAL_FLOW_RESIZE_WIDTH / 2.0
        flow_status = "WATER"
    
        for track in passed_tracks:
            if not track: continue
            current_x = track[-1][0] 
            
            if current_x < center_x:
                left_count += 1
            else:
                right_count += 1
        
        if left_count > 0 and right_count > 0:
            if left_count >= right_count:
                 wall_side = "left"
            else:
                 wall_side = "right"
        elif left_count > 0:
            wall_side = "left"
        elif right_count > 0:
            wall_side = "right"
    else:
        flow_status = "GROUND"
    
    # --- メモリ更新 ---
    expired_cells = []
    for cell in waterfall_memory:
        waterfall_memory[cell] -= 1
        if waterfall_memory[cell] <= 0: 
            expired_cells.append(cell)
    for cell in expired_cells: 
        del waterfall_memory[cell]
    for track in passed_tracks:
        if not track: 
            continue
        end_point = track[-1]
        key = (int(end_point[0]), int(end_point[1]))
        waterfall_memory[key] = DETECTION_TTL
    
    old_gray = frame_gray
    frame_idx += 1
        
    return flow_status,wall_side,old_gray,active_tracks,frame_idx
    

def process_wall_distance(depth_image,camera,roi_y1,roi_y2,roi_w,depth_h,depth_w,side):
    if side =='right':
        roi_x1 = depth_w - roi_w
        roi_x2 = depth_w
    elif side == 'left':
        roi_x1 = 0
        roi_x2 = roi_w
    else:
        return 0.0
    
    if roi_y1 < 0 or roi_y2 > depth_h or roi_x1 < 0 or roi_x2 > depth_w:
        return 0.0

    depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
    non_zero_depth = depth_roi[depth_roi > 0] 

    if non_zero_depth.size > 0:
        return np.mean(non_zero_depth) * camera.depth_scale
    else:
        return 0.0

def calc_avoidance_command(distance,target_wall_side):
    TARGET_DISTANCE = 0.8  # 目標距離 (m)
    ERROR_THRESHOLD = 0.1   # 許容誤差 (m)
        
    if distance == 0.0:
        return "F\n" 
    elif distance is None:
        return "F\n"
    else:
        error = distance - TARGET_DISTANCE
        if abs(error) < ERROR_THRESHOLD:
            command = "F\n" 
        elif error > 0: 
            direction = "Rw" if target_wall_side == 'right' else "Lw"
            command = f"{direction} {abs(error):.2f}\n"
        else: 
            direction = "Lw" if target_wall_side == 'right' else "Rw"
            command = f"{direction} {abs(error):.2f}\n"
            
    return command


def process_cog(color_image, resize_h, debug_info):    
    # 重心検出処理
    STEERING_THRESHOLD = 5 
    
    resized_frame = cv2.resize(color_image, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
    height, width = resized_frame.shape[:2]
    gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
    
    inverted_array = 255 - gray_frame
    total_weight = np.sum(inverted_array)
    image_center_x = width // 2
    center_x = image_center_x
    if total_weight > 0:
        x_coords = np.arange(width)
        center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

    gravity_difference = center_x - image_center_x
    
    # === 描画用データの保存 ===
    # 元画像(640px)のスケールに戻して保存
    orig_h, orig_w = color_image.shape[:2]
    scale = orig_w / width
    debug_info['cog_target_x'] = int(center_x * scale)
    debug_info['cog_center_x'] = int(image_center_x * scale)
    debug_info['cog_y'] = int(height * scale // 2) # 画面中央の高さ
    # ========================

    steering_command = "F\n"
    
    if abs(gravity_difference) > STEERING_THRESHOLD:
        if gravity_difference > 0:
            steering_command = f"R {gravity_difference:.2f}\n" 
        else:
            steering_command = f"L {abs(gravity_difference):.2f}\n"
    
    return steering_command
    
def process_step_detect(depth_image, camera, depth_x_start, depth_x_end, depth_h, depth_w, debug_info):
    #中央短冊の切り出し
    strip_data = depth_image[:, depth_x_start:depth_x_end]    
    
    # 描画用にROI情報を保存 (Color画像上の座標とDepth座標はほぼ一致と仮定、またはアスペクト比で調整)
    debug_info['step_roi'] = (depth_x_start, 0, depth_x_end, depth_h)
    
    row_means = np.mean(strip_data, axis=1) * camera.depth_scale
    with np.errstate(divide='ignore', invalid='ignore'):
        row_means[row_means < 0.1] = np.nan 
        inv_z_profile = 1.0 / row_means
        inv_z_profile = np.nan_to_num(inv_z_profile, nan=0.0, posinf=0.0, neginf=0.0)

    kernel_size = 15
    kernel = np.ones(kernel_size) / kernel_size
    inv_z_smooth = np.convolve(inv_z_profile, kernel, mode='same')
    
    obstacle_detected_start = None 
    
    base_slope_list = []
    check_base_start = depth_h - 20
    check_base_end = depth_h - 40
    
    for k in range(check_base_start, check_base_end, -1):
        val_k = inv_z_smooth[k]
        val_prev = inv_z_smooth[min(depth_h-1, k+5)]
        slope = val_prev - val_k 
        if val_k > 0.5: 
            base_slope_list.append(slope)
    
    ground_slope = np.mean(base_slope_list) if base_slope_list else 0.05
    
    scan_start = depth_h - 45
    scan_end = depth_h // 3
    
    obstacle_pixel_count = 0     
    REQUIRED_HEIGHT_PIXELS = 15  
    
    found_obstacle = False

    for y in range(scan_start, scan_end, -1):
        val = inv_z_smooth[y]
        prev_val = inv_z_smooth[min(depth_h-1, y+5)]
        current_diff = prev_val - val
        is_wall = (current_diff < ground_slope * 0.3) and (val > 0.5)
        
        if is_wall:
            obstacle_pixel_count += 1
        else:
            if obstacle_pixel_count > REQUIRED_HEIGHT_PIXELS:
                obstacle_detected_start = y + obstacle_pixel_count
                found_obstacle = True
                break 
            obstacle_pixel_count = 0
    
    if not found_obstacle and obstacle_detected_start is None and obstacle_pixel_count > REQUIRED_HEIGHT_PIXELS:
        found_obstacle = True

    # 結果を保存
    debug_info['step_detected'] = found_obstacle
    
    return found_obstacle
        
def calc_follow_command(cog_data):
    return "FOLLOW_COMMAND"

def send_motor_command(ser,command):
    # エラー回避のためtry-exceptを追加
    try:
        ser.write(command.encode('utf-8'))
        ser.flush()
        print(f"Sending command: {command}")
    except Exception as e:
        print(f"Serial Error: {e}")


# ==========================================
# 2. メイン制御関数
# ==========================================
def main():
    # --- A. 設定値 (Hz -> 秒換算) ---
    INTERVAL_COG  = 0.500   # 2Hz
    INTERVAL_STEP = 0.500   # 4Hz
    INTERVAL_WALL = 0.066   # 15Hz
    
    LIMIT_TIME_STEP = 0.015 
    LOOP_PERIOD     = 0.025 

    # --- B. 時刻管理変数 (初期化) ---
    last_time_cog = 0
    last_time_step = 0
    last_time_wall = 0
    
    prev_step_data = False
    is_emergency_stop = False
    
    # 画面表示用の状態保持辞書
    debug_info = {
        'cog_target_x': None,
        'cog_center_x': None,
        'cog_y': None,
        'step_roi': None,
        'step_detected': False,
        'last_command': "Init",
        'mode': "Init",
        'fps': 0
    }

    # カメラ起動
    camera = CameraStream()
    camera.start()
    
    print("カメラ初期化中...")
    while True:
        first_color_frame, first_depth_frame, _ = camera.get_latest()
        if first_color_frame is None or first_depth_frame is None:
            time.sleep(0.1)
            continue
        break
    
    orig_h, orig_w = first_color_frame.shape[:2]
    aspect_ratio = orig_h / orig_w
    resize_h = int(RESIZE_WIDTH * aspect_ratio)
    
    depth_h, depth_w = first_depth_frame.shape[:2]
    strip_width = 60
    depth_center_x = depth_w // 2
    depth_x_start = depth_center_x -(strip_width // 2)
    depth_x_end = depth_center_x + (strip_width // 2) 
    
    roi_h = 100
    roi_w = 50
    roi_y1 = (depth_h // 2) - (roi_h // 2)
    roi_y2 = roi_y1 + roi_h
    
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: ポート {SERIAL_PORT} (Baud: {SERIAL_BAUDRATE}) を開きました。")
        time.sleep(2)  
    except serial.SerialException as e:
        print(f"シリアルポートエラー: {e}")
        # テスト用にポートが開けなくても続行させる場合はコメントアウト解除
        # return 
        pass 
    
    optical_flow_resize_height = int(OPTICAL_FLOW_RESIZE_WIDTH * aspect_ratio)
    width_2_9 = OPTICAL_FLOW_RESIZE_WIDTH * 2 // 9
    width_7_9 = OPTICAL_FLOW_RESIZE_WIDTH * 7 // 9
    resized_first_frame = cv2.resize(first_color_frame, (OPTICAL_FLOW_RESIZE_WIDTH, optical_flow_resize_height), interpolation=cv2.INTER_AREA)
    old_gray = cv2.cvtColor(resized_first_frame, cv2.COLOR_BGR2GRAY)
    fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
    
    active_tracks = []
    frame_idx = 0
    waterfall_memory = {}
    
    detect_count = 0
    last_valid_command = "F\n"
    old_flow_result = None
    
    print("制御開始")

    # --- D. メインループ (40Hz) ---
    try:
        while True:
            loop_start_time = time.time()
            
            send_command_flag = False   
            heavy_task_done = False     
            final_cmd = None            

            # 1. 画像取得
            color_img, depth_img, img_time = camera.get_latest()
            if color_img is None: continue

            # 表示用画像のコピー
            display_img = color_img.copy()

            if (loop_start_time - img_time) > 0.1:
                # 遅延時は画面に警告を表示
                cv2.putText(display_img, "DELAY WARNING!", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                debug_info['last_command'] = "STOP(Delay)"
                # continue # 画面表示テスト時はcontinueしない方が見やすい場合もある

            # 2. オプティカルフロー (無効化中だが変数は維持)
            # flow_result,wall_side,old_gray,active_tracks,frame_idx = process_optical_flow(...)
            flow_result = "GROUND"
            debug_info['mode'] = flow_result

            # 3. 状況判断と分岐
            if flow_result == "WATER":
                if (loop_start_time - last_time_wall) >= INTERVAL_WALL:
                    last_time_wall = loop_start_time 
                    send_command_flag = True

            else: # flow_result == "GROUND"
                
                # (1) 重心検出 (2Hz)
                if ((loop_start_time - last_time_cog) >= INTERVAL_COG or old_flow_result == "WATER"):
                    # debug_infoを渡す
                    final_cmd = process_cog(color_img, resize_h, debug_info)
                    
                    last_time_cog = loop_start_time  
                    heavy_task_done = True           
                    send_command_flag = True

                # (2) 段差検出 (4Hz)
                if (loop_start_time - last_time_step) >= INTERVAL_STEP:
                    current_process_time = time.time() - loop_start_time
                    
                    if (not heavy_task_done) and (current_process_time < LIMIT_TIME_STEP):
                        # debug_infoを渡す
                        prev_step_data = process_step_detect(depth_img, camera, depth_x_start, depth_x_end, depth_h, depth_w, debug_info)
                        
                        if prev_step_data == True:
                            detect_count += 1
                        else:
                            detect_count = 0
                        last_time_step = loop_start_time 
                        send_command_flag = True
                    else:
                        pass

            # コマンド決定
            if send_command_flag == True and prev_step_data == True and detect_count >= 3:
                final_cmd = "S\n"  
                is_emergency_stop = True
                debug_info['mode'] = "EMERGENCY"
            elif is_emergency_stop:
                final_cmd ="S\n"
                send_command_flag = True
                debug_info['mode'] = "EMERGENCY"

            # 4. コマンド送信
            if send_command_flag and final_cmd:
                send_motor_command(ser, final_cmd)
                debug_info['last_command'] = final_cmd.strip() # 改行削除
                if final_cmd != "S\n":
                    last_valid_command = final_cmd
                    old_flow_result = flow_result

            # ==============================================
            # 5. 画面描画処理 (追加部分)
            # ==============================================
            
            # (A) 重心情報の描画
            if debug_info['cog_target_x'] is not None:
                cx = debug_info['cog_center_x']
                tx = debug_info['cog_target_x']
                cy = debug_info['cog_y']
                # 中心線 (白)
                cv2.line(display_img, (cx, 0), (cx, orig_h), (200, 200, 200), 1)
                # ターゲット重心 (青丸)
                cv2.circle(display_img, (tx, cy), 8, (255, 0, 0), -1)
                # 中心からターゲットへの線 (黄色)
                cv2.line(display_img, (cx, cy), (tx, cy), (0, 255, 255), 2)

            # (B) 段差検出エリアの描画
            if debug_info['step_roi'] is not None:
                sx1, sy1, sx2, sy2 = debug_info['step_roi']
                # 検出状態によって色を変える (検出=赤, 安全=緑)
                color = (0, 0, 255) if debug_info['step_detected'] else (0, 255, 0)
                cv2.rectangle(display_img, (sx1, sy1), (sx2, sy2), color, 2)
                
                # 検出カウントの表示
                if detect_count > 0:
                    cv2.putText(display_img, f"Alert: {detect_count}", (sx1, sy1+20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # (C) ステータス情報のオーバーレイ
            fps = 1.0 / (time.time() - loop_start_time + 0.0001)
            cv2.putText(display_img, f"CMD: {debug_info['last_command']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display_img, f"Mode: {debug_info['mode']}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            
            # 画像表示
            cv2.imshow("Robot View", display_img)
            # キー入力待機 (これがないとウィンドウが更新されない)
            key = cv2.waitKey(1)
            if key == 27: # ESCキーで終了
                break

            # ==============================================

            # 6. 周期調整
            elapsed = time.time() - loop_start_time
            sleep_time = LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("停止シグナル受信")
    finally:
        # 終了処理
        send_motor_command(ser, "S\n") # 安全のため停止
        cv2.destroyAllWindows()
        print("終了しました。")

if __name__ == "__main__":
    main()