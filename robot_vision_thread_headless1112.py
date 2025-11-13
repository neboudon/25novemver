# ファイル名: robot_vision_thread_headless1.py (アーキテクチャ変更版)

import cv2
import numpy as np
import math
import time

# --- パラメータ (軽量化のため調整) ---
RESIZE_WIDTH = 240
MIN_NOISE_AREA = 45 

# ===================================================================
# スレッド 1: カメラ取得用スレッド
# ===================================================================
def camera_capture_thread(camera_index, shared_state, lock):
    """
    【新設スレッド】
    指定されたFPSでカメラからフレームを取得し、
    'latest_frame' として共有辞書に書き込むだけ。
    """
    # このスレッドが画像処理全体のFPSを決定する
    TARGET_FPS = 5  # (線検出 2FPS + 重心 5FPS だったので、中間的な 5FPS を選択)
    INTERVAL = 1.0 / TARGET_FPS

    print(f"[カメラ取得スレッド]: カメラ({camera_index})の起動を試みます...")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[カメラ取得スレッド] エラー: カメラ ({camera_index}) を開けません。")
        with lock:
            shared_state['stop'] = True 
        return
    print(f"[カメラ取得スレッド]: カメラ({camera_index}) 起動完了。")

    last_processed_time = time.time()

    while True:
        with lock:
            if shared_state['stop']:
                break
        
        ret, frame = cap.read()
        if not ret:
            print("[カメラ取得スレッド] エラー: フレームを取得できません。")
            time.sleep(0.5)
            continue

        current_time = time.time()
        
        # ターゲットFPSになるように待機
        if (current_time - last_processed_time) >= INTERVAL:
            last_processed_time = current_time
            
            with lock:
                # 最新のフレームを共有辞書に上書き
                shared_state['latest_frame'] = frame
                # 新しいフレームが来たことを処理スレッドに通知
                shared_state['new_frame_flag'] = True 

        # CPUを過剰に消費しないよう、少し待機
        time.sleep(0.005) 

    cap.release()
    print("[カメラ取得スレッド]: カメラを解放しました。")


# ===================================================================
# スレッド 2: 画像処理スレッド (操舵 + 重心)
# ===================================================================
def vision_processing_thread(shared_state, lock):
    """
    【統合スレッド】
    'latest_frame' が更新されたら、それを使って処理を実行する。
    1. 操舵（消失点）検出を試みる。
    2. 失敗した場合のみ、同じフレームで重心検出を実行する。
    """
    
    # --- 操舵用パラメータ ---
    CANNY_THRESHOLD1 = 100
    CANNY_THRESHOLD2 = 150
    HOUGH_THRESHOLD = 35 
    HOUGH_MIN_LINE_LENGTH = 35
    HOUGH_MAX_LINE_GAP = 10
    CLIP_LIMIT = 15.0
    TILE_GRID_SIZE = (4, 4)
    
    # (重心検出用のパラメータは特になし)

    print(f"[画像処理スレッド]: 待機中...")
    
    while True:
        with lock:
            if shared_state['stop']:
                break
            
            # 新しいフレームが来ていなければ、処理をスキップ
            if not shared_state.get('new_frame_flag', False):
                continue
            
            # フラグを下げ、フレームを取得
            shared_state['new_frame_flag'] = False
            frame = shared_state['latest_frame']
            if frame is None:
                continue
        
        # --- 処理開始 (ロックの外側で重い処理を実行) ---
        try:
            # === 1. リサイズと前処理 (共通) ===
            orig_height, orig_width = frame.shape[:2]
            aspect_ratio = orig_height / orig_width
            resize_height = int(RESIZE_WIDTH * aspect_ratio)
            resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
            height, width = resized_frame.shape[:2]
            image_center_x = width / 2
            
            # === 2. 操舵（消失点）検出 ===
            
            # 操舵用の前処理
            gray_steering = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
            adjusted = clahe.apply(gray_steering)
            blurred_again = cv2.GaussianBlur(adjusted, (7, 7), 0)
            edges = cv2.Canny(blurred_again, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
            
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edges, connectivity=8)
            cleaned_edges = np.zeros_like(edges)
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] > MIN_NOISE_AREA:
                    cleaned_edges[labels == i] = 255
            
            lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi/180,
                                    threshold=HOUGH_THRESHOLD,
                                    minLineLength=HOUGH_MIN_LINE_LENGTH,
                                    maxLineGap=HOUGH_MAX_LINE_GAP)
            
            diagonal_lines = []
            vp_x = width // 2
            steering_success = False # 検出フラグを初期化

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
                steering_success = True # 消失点が見つかった！

            # --- ズレ量を計算 ---
            steering_difference = vp_x - image_center_x

            # === 3. フォールバック処理 (重心検出) ===
            
            if not steering_success:
                # 操舵が失敗した場合のみ、**同じフレーム** (`resized_frame`) を使って重心を計算
                
                # 重心用の前処理 (グレー化)
                gray_frame_gravity = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

                # 重心計算
                inverted_array = 255 - gray_frame_gravity
                total_weight = np.sum(inverted_array)
                center_x = width / 2 
                if total_weight > 0:
                    x_coords = np.arange(width)
                    center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

                # 重心のズレ量を計算
                gravity_difference = center_x - image_center_x
                
                # 結果を共有辞書に書き込み
                with lock:
                    shared_state['steering_success'] = False
                    # 操舵が失敗したので、'gravity_value' に重心のズレを入れる
                    shared_state['gravity_value'] = gravity_difference
                    # 'steering_value' も念のためリセット（または前回の値を維持）
                    shared_state['steering_value'] = 0.0 
            
            else:
                # 操舵が成功した場合
                with lock:
                    shared_state['steering_success'] = True
                    # 'steering_value' に操舵のズレを入れる
                    shared_state['steering_value'] = steering_difference
                    # 'gravity_value' は計算していないので更新しない
                    
        except Exception as e:
            print(f"[画像処理スレッド] 処理中に予期せぬエラー: {e}")
            pass # ループを継続
        
        # 次のフレームフラグが立つまで待機
        time.sleep(0.005) 

    print("[画像処理スレッド]: 終了しました。")


# ===================================================================
# スレッド3: 壁検出用 (メインからは呼び出されていない)
# ===================================================================
def wall_thread_func(camera_index, shared_state, lock):
    # (この関数はメインから呼び出されないが、念のため残しておく)
    print("[壁検出スレッド]: 起動しましたが、このスクリプトでは使用されません。")
    pass