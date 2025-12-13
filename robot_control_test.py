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
SERIAL_BAUDRATE = 921600  # Pico側もこれに合わせてください

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
        # RealSense初期化 (60FPS)
        # 【重要】内部キューを1にして、古い画像を溜めない
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
        #self.pipeline.start(self.config)
        profile = self.pipeline.start(self.config)
        #self.depth_sensor = self.pipeline.get_device().first_depth_sensor()
        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.capture_time = 0

    def start(self):
        # 別スレッドを開始
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while True:
            # 常に全力で回して、変数を上書きし続ける
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            #color_frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            #depth_frame = self.pipeline.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            now = time.time()
            
            with self.lock:
                self.latest_color_frame = color_image
                self.latest_depth_frame = depth_image
                self.capture_time = now

    def get_latest(self):
        # メイン処理が呼び出す関数
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
        # old_gray (前回処理したフレーム) と frame_gray (現在) で比較
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        for i, (track, (new_point, status)) in enumerate(zip(active_tracks, zip(p1, st))):
            if status[0] == 0:
                continue
            
            new_point_tuple = tuple(new_point.ravel())
            new_x = new_point_tuple[0]
            
            # 領域外（中央部分）なら追跡中止
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
        
        #for track in active_tracks: 
        #    if track:
        #        cv2.circle(detection_mask, (int(track[-1][0]), int(track[-1][1])), NEW_POINT_MIN_DIST, 0, -1)
        
        keypoints = fast.detect(frame_gray, mask=detection_mask)
        #print("keypoints",keypoints)
        if keypoints:
            # 3. 【重要】スコア（強さ）順にソートして、上位のみ採用する
            # これをしないと、水面のノイズなどで点が500個とか見つかってしまい、
            # 次のオプティカルフロー計算でFPSが激減します。
            keypoints = sorted(keypoints, key=lambda x: x.response, reverse=True)
            
            # feature_params['maxCorners'] (例: 100個) だけ採用
            limit_count = feature_params['maxCorners'] 
            best_keypoints = keypoints[:limit_count]

            # 4. 座標を取り出して active_tracks に追加
            for kp in best_keypoints:
                x, y = kp.pt # float型
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
    
    # ============================================================
    wall_side = "none" # デフォルト値
    
    if passed_tracks:
        left_count = 0
        right_count = 0
        center_x = OPTICAL_FLOW_RESIZE_WIDTH / 2.0  # 画像の中心X座標
        flow_status = "WATER"
    
        for track in passed_tracks:
            if not track: continue
            # トラックの最新点(または平均点)のX座標を取得
            current_x = track[-1][0] 
            
            if current_x < center_x:
                left_count += 1
            else:
                right_count += 1
        
        # 判定結果を wall_side に格納
        if left_count > 0 and right_count > 0:
            # 両方に検出された場合（前方全体が崖、または両側に崖）
            # 必要に応じて "both" や数の多い方を採用するなど調整してください
            if left_count >= right_count:
                 wall_side = "left" # あるいは "both"
            else:
                 wall_side = "right"
        elif left_count > 0:
            wall_side = "left"
        elif right_count > 0:
            wall_side = "right"

    # ============================================================
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
    
    # 次の比較用に現在のグレー画像を保存
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
    
    # ROI抽出と計算
    if roi_y1 < 0 or roi_y2 > depth_h or roi_x1 < 0 or roi_x2 > depth_w:
        return 0.0 # 範囲外安全策

    depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
    non_zero_depth = depth_roi[depth_roi > 0] # 0 (測定不能) を除外

    if non_zero_depth.size > 0:
        return np.mean(non_zero_depth) * camera.depth_scale
    else:
        return 0.0

def calc_avoidance_command(distance,target_wall_side):
    # 壁追従コマンド計算のパラメータ
    TARGET_DISTANCE = 0.8  # 目標距離 (m)
    ERROR_THRESHOLD = 0.1   # 許容誤差 (m)
        
    if distance == 0.0:
        return "F\n"  # 停止コマンド（距離不明）
    elif distance is None:
        return "F\n"  # 停止コマンド（距離不明）
    else:
        error = distance - TARGET_DISTANCE
        if abs(error) < ERROR_THRESHOLD:
            command = "F\n" 
            print(f" [CONTROL] OK ({distance:.2f}m) -> 'S'")
        elif error > 0: # 遠い
            # 右壁ターゲットで遠い(右に寄りたい) -> Right
            # 左壁ターゲットで遠い(左に寄りたい) -> Left
            direction = "Rw" if target_wall_side == 'right' else "Lw"
            command = f"{direction} {abs(error):.2f}\n"
            print(f" [CONTROL] 遠い ({distance:.2f}m) -> '{direction}'")
        else: # 近い (error < 0)
            # 右壁ターゲットで近い(左に避けたい) -> Left
            # 左壁ターゲットで近い(右に避けたい) -> Right
            direction = "Lw" if target_wall_side == 'right' else "Rw"
            command = f"{direction} {abs(error):.2f}\n"
            print(f" [CONTROL] 近い ({distance:.2f}m) -> '{direction}'")
            
    return command


def process_cog(color_image,resize_h):    
    # 重心検出処理の変数の定義
    STEERING_THRESHOLD = 5  # 重心差の閾値（ピクセル）
    
    # リサイズとグレースケール変換
    resized_frame = cv2.resize(color_image, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
    height, width = resized_frame.shape[:2]
    gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
    
    #暗点重心の検出
    inverted_array = 255 - gray_frame
    total_weight = np.sum(inverted_array)
    image_center_x = width // 2
    center_x = image_center_x
    if total_weight > 0:
        x_coords = np.arange(width)
        center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

    gravity_difference = center_x - image_center_x
    
    steering_command = "F\n"
    
    if abs(gravity_difference) > STEERING_THRESHOLD:
        if gravity_difference > 0:
            steering_command = f"R {gravity_difference:.2f}\n" 
        else:
            steering_command = f"L {abs(gravity_difference):.2f}\n"
    
    return steering_command
    
def process_step_detect(depth_image,camera,depth_x_start,depth_x_end,depth_h,depth_w):
    #中央短冊の切り出し
    strip_data = depth_image[:, depth_x_start:depth_x_end]    
    
    #1/zプロファイルの作成
    row_means = np.mean(strip_data, axis=1) * camera.depth_scale
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
    check_base_start = depth_h - 20
    check_base_end = depth_h - 40
    
    for k in range(check_base_start, check_base_end, -1):
        val_k = inv_z_smooth[k]
        val_prev = inv_z_smooth[min(depth_h-1, k+5)]
        slope = val_prev - val_k # 正の値になるはず（手前の方が値が大きいから）
        if val_k > 0.5: # データが有効な場合のみ
            base_slope_list.append(slope)
    
    # 基準勾配 (デフォルトは0.05程度と仮定)
    ground_slope = np.mean(base_slope_list) if base_slope_list else 0.05
    
    # (B) 探索ループ
    scan_start = depth_h - 45
    scan_end = depth_h // 3
    
    obstacle_pixel_count = 0     # 壁らしきものが何ピクセル続いたか
    REQUIRED_HEIGHT_PIXELS = 15  # 何ピクセル続いたら「乗り越え不可」とみなすか（調整箇所）
    
    for y in range(scan_start, scan_end, -1):
        val = inv_z_smooth[y]
        prev_val = inv_z_smooth[min(depth_h-1, y+5)]
        
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
                #obstacle_detected_end = y
                return True # 一番手前の障害物を見つけたら終了
            
            # 十分な高さがなければリセット（ただの小石やノイズ）
            obstacle_pixel_count = 0
    
    # ループを抜けた後、画面端まで壁が続いていた場合の処理
    if obstacle_detected_start is None and obstacle_pixel_count > REQUIRED_HEIGHT_PIXELS:
        return True
    return False
        
def calc_follow_command(cog_data):
    # ダミー関数: 実際の追従コマンド計算をここに実装
    return "FOLLOW_COMMAND"

def send_motor_command(ser,command):
    ser.write(command.encode('utf-8'))
    ser.flush()
    print(f"Sending command: {command}")


# ==========================================
# 2. メイン制御関数
# ==========================================
def main():
    # --- A. 設定値 (Hz -> 秒換算) ---
    INTERVAL_COG  = 0.500   # 2Hz
    INTERVAL_STEP = 0.500   # 4Hz
    INTERVAL_WALL = 0.066   # 15Hz
    
    LIMIT_TIME_STEP = 0.015 # 残り時間がこれ以下なら段差検出はやらない(秒)
    LOOP_PERIOD     = 0.025 # 40Hzの基本周期

    # --- B. 時刻管理変数 (初期化) ---
    last_time_cog = 0
    last_time_step = 0
    last_time_wall = 0
    
    # --- C. 前回の結果保持 ---
    #prev_cog_data = None
    prev_step_data = False
    
    is_emergency_stop = False
    
    # カメラ起動
    camera = CameraStream()
    camera.start()
    
    while True:
        first_color_frame, first_depth_frame, _ = camera.get_latest()
        if first_color_frame is None or first_depth_frame is None:
            print("Failed to get first frame from camera.")
            time.sleep(0.1)
            continue
        break
    
    # 重心検出用のリサイズ後の高さ計算
    orig_h, orig_w = first_color_frame.shape[:2]
    aspect_ratio = orig_h / orig_w
    resize_h = int(RESIZE_WIDTH * aspect_ratio)
    
    #段差検出のためのパラメータ計算
    depth_h, depth_w = first_depth_frame.shape[:2]
    strip_width = 60
    depth_center_x = depth_w // 2
    depth_x_start = depth_center_x -(strip_width // 2)
    depth_x_end = depth_center_x + (strip_width // 2) 
    
    #壁距離計算のためのパラメータ計算
    roi_h = 100
    roi_w = 50
    roi_y1 = (depth_h // 2) - (roi_h // 2)
    roi_y2 = roi_y1 + roi_h
    
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: ポート {SERIAL_PORT} (Baud: {SERIAL_BAUDRATE}) を開きました。")
        time.sleep(2)  # シリアルポートの初期化待ち
    
    except serial.SerialException as e:
        print("シリアルポートのオープンに失敗しました。")
        return
    
    #オプティカルフローの処理パラメータの計算
    optical_flow_resize_height = int(OPTICAL_FLOW_RESIZE_WIDTH * aspect_ratio)
    width_2_9 = OPTICAL_FLOW_RESIZE_WIDTH * 2 // 9
    width_7_9 = OPTICAL_FLOW_RESIZE_WIDTH * 7 // 9
    resized_first_frame = cv2.resize(first_color_frame, (OPTICAL_FLOW_RESIZE_WIDTH, optical_flow_resize_height), interpolation=cv2.INTER_AREA)
    old_gray = cv2.cvtColor(resized_first_frame, cv2.COLOR_BGR2GRAY)
    fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
    
    # オプティカルフロー関連の変数初期化
    active_tracks = []
    frame_idx = 0
    waterfall_memory = {}
    detection_log = []#これをどこで使用する？
    
    #段差検出のための変数
    detect_count = 0
    
    #コマンド送信しない時のコマンド
    last_valid_command = "F\n"
    old_flow_result = None
    
    print("制御開始")

    # --- D. メインループ (40Hz) ---
    while True:
        loop_start_time = time.time()
        
        # フラグ初期化
        send_command_flag = False   # 今回コマンドを送るか？
        heavy_task_done = False     # 今回重い処理をしたか？
        final_cmd = None            # 送信するコマンド内容

        # ------------------------------------------------
        # 1. 画像取得 (スレッドから最新を取得)
        # ------------------------------------------------
        color_img,depth_img, img_time = camera.get_latest()
        
        # 安全装置: カメラ映像が古すぎる(0.1秒以上)場合は停止
        if (loop_start_time - img_time) > 0.1:
            print("警告: 映像遅延。停止します。")
            continue

        # ------------------------------------------------
        # 2. オプティカルフロー (毎回実行)
        # ------------------------------------------------
        #flow_result,wall_side,old_gray,active_tracks,frame_idx = process_optical_flow(color_img,optical_flow_resize_height,width_2_9,width_7_9,fast,waterfall_memory,old_gray,active_tracks,frame_idx)
        flow_result = "GROUND"
        # ------------------------------------------------
        # 3. 状況判断と分岐
        # ------------------------------------------------
        
        # === ケースA: 水面を検出 ===
        if flow_result == "WATER":
            
            # 15Hz (0.066秒) 経過したかチェック
            if (loop_start_time - last_time_wall) >= INTERVAL_WALL:
                #dist = process_wall_distance(depth_img,camera,roi_y1,roi_y2,roi_w,depth_h,depth_w,wall_side)
                
                last_time_wall = loop_start_time # ★時刻更新
                
                # コマンド生成
                #final_cmd = calc_avoidance_command(dist,wall_side)
                send_command_flag = True

        # === ケースB: 地面 ===
        else: # flow_result == "GROUND"
            
            # (1) 重心検出チェック (2Hz)
            if ((loop_start_time - last_time_cog) >= INTERVAL_COG or old_flow_result == "WATER"):
                final_cmd = process_cog(color_img,resize_h)
                
                last_time_cog = loop_start_time  # ★時刻更新
                heavy_task_done = True           # 「重い処理しました」
                send_command_flag = True

            # (2) 段差検出チェック (4Hz)
            #  「時間が来ている」 かつ 「さっき重心検出をしていない」
            if (loop_start_time - last_time_step) >= INTERVAL_STEP:
                
                current_process_time = time.time() - loop_start_time
                
                # さらに「残り時間に余裕があるか」チェック
                if (not heavy_task_done) and (current_process_time < LIMIT_TIME_STEP):
                    #段差の検出の関数
                    prev_step_data = process_step_detect(depth_img,camera,depth_x_start,depth_x_end,depth_h,depth_w)
                    
                    if prev_step_data == True:
                        detect_count+=1
                    else:
                        detect_count = 0
                    last_time_step = loop_start_time # ★時刻更新
                    send_command_flag = True
                
                else:
                    # ここに来た＝「やる予定だったけど忙しいからスキップした」
                    # ★重要: last_time_step を更新しない！
                    # 次のループで即座に再チャレンジさせる
                    pass

        # コマンド生成
        if send_command_flag == True and prev_step_data == True and detect_count >= 3:
            final_cmd = "S\n"  # 停止コマンド
            is_emergency_stop = True
        elif is_emergency_stop:
            final_cmd ="S\n"
            send_command_flag = True
        #elif send_command_flag is not True and final_cmd is None:
        #    final_cmd = last_valid_command#calc_follow_command(prev_cog_data)#この関数いらないかも
        #    send_command_flag = True
        # ------------------------------------------------
        # 4. コマンド送信
        # ------------------------------------------------
        if send_command_flag and final_cmd:
            send_motor_command(ser,final_cmd)
            if final_cmd != "S\n":
                last_valid_command = final_cmd
                old_flow_result = flow_result

        # ------------------------------------------------
        # 5. 周期調整 (40Hzを守る)
        # ------------------------------------------------
        elapsed = time.time() - loop_start_time
        sleep_time = LOOP_PERIOD - elapsed
        
        if sleep_time > 0:
            time.sleep(sleep_time)
            
if __name__ == "__main__":
    main()