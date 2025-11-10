# ファイル名: robot_vision_thread_headless.py
# (main_control_thread_serial.py と同じフォルダに保存してください)

import cv2
import numpy as np
import math
import time

# --- パラメータ (軽量化のため調整) ---
RESIZE_WIDTH = 240
MIN_NOISE_AREA = 45 

# ===================================================================
# スレッド1: 操舵用 (描画・フレーム共有を無効化)
# ===================================================================
def steering_thread_func(camera_index, shared_state, lock):
    """
    【スレッド版・ヘッドレス】
    操舵（消失点）を検出し、ズレ量(float)のみを共有辞書に書き込む。
    """
    
    # --- 操舵用パラメータ ---
    CANNY_THRESHOLD1 = 100
    CANNY_THRESHOLD2 = 150
    HOUGH_THRESHOLD = 35 
    HOUGH_MIN_LINE_LENGTH = 35
    HOUGH_MAX_LINE_GAP = 10
    CLIP_LIMIT = 15.0
    TILE_GRID_SIZE = (4, 4)
    TARGET_FPS = 2  
    INTERVAL = 1.0 / TARGET_FPS

    print(f"[操舵スレッド]: カメラ({camera_index})の起動を試みます...")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[操舵スレッド] エラー: カメラ ({camera_index}) を開けません。")
        with lock:
            shared_state['stop'] = True 
        return
    print(f"[操舵スレッド]: カメラ({camera_index}) 起動完了。")
    
    last_processed_time = time.time()
    
    # --- メインループ ---
    while True:
        with lock:
            if shared_state['stop']:
                break
                
        ret, frame = cap.read()
        if not ret:
            print("[操舵スレッド] エラー: フレームを取得できません。")
            time.sleep(0.5)
            continue
        
        current_time = time.time()
        
        if(current_time - last_processed_time) >= INTERVAL:
            
            last_processed_time = current_time
        
            # --- 1. リサイズ ---
            try:
                orig_height, orig_width = frame.shape[:2]
                aspect_ratio = orig_height / orig_width
                resize_height = int(RESIZE_WIDTH * aspect_ratio)
                resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
                height, width = resized_frame.shape[:2]
                
                # --- 2. 前処理 ---
                gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
                clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
                adjusted = clahe.apply(gray)
                blurred_again = cv2.GaussianBlur(adjusted, (7, 7), 0)
                edges = cv2.Canny(blurred_again, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
                
                # --- 3. ノイズ除去 ---
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edges, connectivity=8)
                cleaned_edges = np.zeros_like(edges)
                for i in range(1, num_labels):
                    if stats[i, cv2.CC_STAT_AREA] > MIN_NOISE_AREA:
                        cleaned_edges[labels == i] = 255
                
                # --- 4. ハフ変換 & 消失点計算 ---
                lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi/180,
                                        threshold=HOUGH_THRESHOLD,
                                        minLineLength=HOUGH_MIN_LINE_LENGTH,
                                        maxLineGap=HOUGH_MAX_LINE_GAP)
                
                diagonal_lines = []
                vp_x = width // 2
                
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

                        # --- (描画処理 コメントアウト) ---
                        # points = []
                        # if m != 0:
                        #     x_at_y0 = -c / m
                        #     if 0 <= x_at_y0 <= width: points.append((int(x_at_y0), 0))
                        #     x_at_y_height = (height - c) / m
                        #     if 0 <= x_at_y_height <= width: points.append((int(x_at_y_height), height))
                        # y_at_x0 = c
                        # if 0 <= y_at_x0 <= height: points.append((0, int(y_at_x0)))
                        # y_at_x_width = m * width + c
                        # if 0 <= y_at_x_width <= height: points.append((width, int(y_at_x_width)))
                    
                        # if len(points) >= 2:
                        #     cv2.line(resized_frame, points[0], points[1], (0, 255, 0), 2)
                        # --- (描画処理 コメントアウトここまで) ---


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

                # --- 5. ズレ量を計算 ---
                image_center_x = width / 2
                x_difference = vp_x - image_center_x
                
                # --- 5b. デバッグ描画 (コメントアウト) ---
                # cv2.circle(resized_frame, (vp_x, height // 2), 10, (0, 0, 255), -1) 
                # cv2.line(resized_frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 1)

                # --- 6. 共有辞書へ書き込み (ロックを使用) ---
                with lock:
                    shared_state['steering_value'] = x_difference
                    # shared_state['steering_frame'] = resized_frame.copy() # ★★★ GUI用にコメントアウト ★★★
            except Exception as e:
                    print(f"[操舵スレッド] 処理中に予期せぬエラー: {e}")
                    pass
        
        time.sleep(0.001) 

    cap.release()
    print("[操舵スレッド]: カメラを解放しました。")


# ===================================================================
# スレッド2: 壁検出用 (描画・フレーム共有を無効化)
# ===================================================================
def wall_thread_func(camera_index, shared_state, lock):
    """
    【スレッド版・360度カメラ対応・ヘッドレス】
    壁を検出し、フラグのみを共有辞書に書き込む。
    """

    # --- 壁検出用パラメータ ---
    CANNY_THRESHOLD1 = 90 
    CANNY_THRESHOLD2 = 150
    HOUGH_THRESHOLD = 30 
    HOUGH_MIN_LINE_LENGTH = 30
    HOUGH_MAX_LINE_GAP = 10
    CLIP_LIMIT = 15.0
    TILE_GRID_SIZE = (12, 12)
    TARGET_FPS = 5  
    INTERVAL = 1.0 / TARGET_FPS
    
    print(f"[壁検出スレッド]: カメラ({camera_index})の起動を試みます...")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[壁検出スレッド] エラー: カメラ ({camera_index}) を開けません。")
        with lock:
            shared_state['stop'] = True 
        return
    print(f"[壁検出スレッド]: カメラ({camera_index}) 起動完了。")

    last_processed_time = time.time()
    
    # --- メインループ ---
    while True:
        with lock:
            if shared_state['stop']:
                break

        ret, frame = cap.read()
        if not ret or frame is None:
            print("[壁検出スレッド] エラー: フレームを取得できません。")
            time.sleep(0.5)
            continue
        
        current_time = time.time()
        
        if(current_time - last_processed_time) >= INTERVAL:
            
            last_processed_time = current_time
        
            # wall_line_detected = 0 # ★★★ 変更 ★★★
            right_fragment_detected = 0 # 右側ビュー（上半分）での検出フラグ
            left_fragment_detected = 0  # 左側ビュー（下半分）での検出フラグ
            # processed_frames = [] # ★★★ GUI用にコメントアウト ★★★
            
            # --- 1. フレームを上下（右と左）に分割 ---
            try:
                orig_height, orig_width = frame.shape[:2]
                half_height = orig_height // 2
                
                frame_right = frame[0:half_height, :] #上半分=右
                frame_left = frame[half_height:, :] #下半分=左
                
                fragments = [frame_right, frame_left]
            
            except Exception as e:
                print(f"[壁検出スレッド] フレーム分割エラー: {e}")
                continue
                
            # --- 2. 分割した画像を個別に処理 ---
            for i, img_fragment in enumerate(fragments):
                
                is_right_image = (i == 0)
                
                # fragment_detected = 0 # ★★★ GUI用にコメントアウト ★★★
                resized_frame = None 
                width, height = 0, 0 # スコープのために初期化

                try:
                    # --- 2a. リサイズ ---
                    orig_h_frag, orig_w_frag = img_fragment.shape[:2]
                    if orig_h_frag == 0 or orig_w_frag == 0:
                        raise ValueError("Fragment is empty")
                        
                    aspect_ratio = orig_h_frag / orig_w_frag
                    resize_height = int(RESIZE_WIDTH * aspect_ratio)
                    resized_frame = cv2.resize(img_fragment, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
                    height, width = resized_frame.shape[:2] 

                    # --- 2b. 前処理 ---
                    gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
                    clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
                    adjusted = clahe.apply(gray)
                    blurred_again = cv2.GaussianBlur(adjusted, (7, 7), 0)
                    edges = cv2.Canny(blurred_again, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
                    
                    # --- 2d. ハフ変換 ---
                    lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                            threshold=HOUGH_THRESHOLD,
                                            minLineLength=HOUGH_MIN_LINE_LENGTH,
                                            maxLineGap=HOUGH_MAX_LINE_GAP)
                    
                    # --- 2e. 検出ロジック ---
                    if lines is not None:
                        image_center_x = width / 2 # イメージ中心
                        
                        for line in lines:
                            x1, y1, x2, y2 = line[0]
                            angle_deg = math.degrees(math.atan2(y2 - y1, x2 - x1))
                            abs_angle_deg = abs(angle_deg)
                            
                            is_vertical = (80 <= abs_angle_deg <= 100)
                            
                            if is_vertical:
                                line_center_x = (x1 + x2) / 2
                                
                                detection_condition_met = False
                                if is_right_image:
                                    if line_center_x > image_center_x: # 右側ビューで右に線
                                        detection_condition_met = True
                                else:
                                    if line_center_x < image_center_x: # 左側ビューで左に線
                                        detection_condition_met = True
                                
                                if detection_condition_met:
                                    # fragment_detected = 1 # ★★★ GUI用にコメントアウト ★★★
                                    
                                    # ★★★ 変更: 左右別々にフラグを立てる ★★★
                                    if is_right_image:
                                        right_fragment_detected = 1
                                    else:
                                        left_fragment_detected = 1
                                    
                                    # --- (描画処理 コメントアウト) ---
                                    # start_point, end_point = None, None
                                    # if x1 == x2:  
                                    #     start_point = (x1, 0)
                                    #     end_point = (x1, height)
                                    # else:  
                                    #     m = (y2 - y1) / (x2 - x1)
                                    #     c = y1 - m * x1
                                    #     start_point = (0, int(c))
                                    #     end_point = (width, int(m * width + c))
                                    # cv2.line(resized_frame, start_point, end_point, (0, 0, 255), 2)
                                    # --- (描画処理 コメントアウトここまで) ---
                                    break # 1本見つかればOK
                
                except Exception as e:
                    print(f"[壁検出スレッド] Fragment処理エラー: {e}")
                    if width == 0: width = RESIZE_WIDTH
                    if height == 0: height = int(RESIZE_WIDTH * (3/4))
                    # resized_frame = np.zeros((height, width, 3), dtype=np.uint8) # ★★★ GUI用にコメントアウト ★★★


                # --- 2f. 描画 (コメントアウト) ---
                # if width > 0 and height > 0:
                #     cv2.line(resized_frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 1) 
                
                # processed_frames.append(resized_frame) # ★★★ GUI用にコメントアウト ★★★

            # --- 3. 処理済みフレームを *別々に* 共有 ★ ---
            try:
                with lock:
                    # ★★★ 変更: 両方が1の場合のみ、'wall_detected'を1にする ★★★
                    if right_fragment_detected == 1 and left_fragment_detected == 1:
                        shared_state['wall_detected'] = 1
                    else:
                        shared_state['wall_detected'] = 0
                    
                    # shared_state['wall_frame_right'] = processed_frames[0].copy() # ★★★ GUI用にコメントアウト ★★★
                    # shared_state['wall_frame_left'] = processed_frames[1].copy() # ★★★ GUI用にコメントアウト ★★★
            
            except Exception as e:
                # processed_frames が空の場合に Index Error が発生する可能性があるが、
                # GUIを無効化したため、このエラーは発生しなくなる
                print(f"[壁検出スレッド] 共有辞書への保存エラー: {e}")
            
        time.sleep(0.001) 

    cap.release()
    print("[壁検出スレッド]: カメラを解放しました。")
    

# ===================================================================
# スレッド3: 重心検出用 (描画・フレーム共有を無効化)
# ===================================================================
def gravity_thread_func(camera_index, shared_state, lock):
    """
    【スレッド版・ヘッドレス】
    最も暗い部分の重心を検出し、ズレ量(float)のみを共有辞書に書き込む。
    """
    
    # --- 重心検出用パラメータ ---
    TARGET_FPS = 5  
    INTERVAL = 1.0 / TARGET_FPS

    print(f"[重心スレッド]: カメラ({camera_index})の起動を試みます...")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[重心スレッド] エラー: カメラ ({camera_index}) を開けません。")
        with lock:
            shared_state['stop'] = True 
        return
    print(f"[重心スレッド]: カメラ({camera_index}) 起動完了。")
    
    last_processed_time = time.time()
    
    # --- メインループ ---
    while True:
        with lock:
            if shared_state['stop']:
                break
                
        ret, frame = cap.read()
        if not ret:
            print("[重心スレッド] エラー: フレームを取得できません。")
            time.sleep(0.5)
            continue
        
        current_time = time.time()
        
        if(current_time - last_processed_time) >= INTERVAL:
            
            last_processed_time = current_time
        
            try:
                # --- 1. リサイズ ---
                orig_height, orig_width = frame.shape[:2]
                aspect_ratio = orig_height / orig_width
                resize_height = int(RESIZE_WIDTH * aspect_ratio)
                resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
                height, width = resized_frame.shape[:2]

                # --- 2. グレースケールに変換 ---
                gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

                # --- 3. 重心計算 ---
                inverted_array = 255 - gray_frame
                total_weight = np.sum(inverted_array)
                center_x = width / 2 
                if total_weight > 0:
                    x_coords = np.arange(width)
                    center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

                # --- 4. ズレ量を計算 ---
                image_center_x = width / 2
                x_difference = center_x - image_center_x
                
                # --- 5. デバッグ描画 (コメントアウト) ---
                # cv2.line(resized_frame, (width // 2, 0), (width // 2, height), (255, 255, 0), 2)
                # cv2.circle(resized_frame, (int(center_x), height // 2), 10, (0, 0, 255), -1)

                # --- 6. 共有辞書へ書き込み (ロックを使用) ---
                with lock:
                    shared_state['gravity_value'] = x_difference
                    # shared_state['gravity_frame'] = resized_frame.copy() # ★★★ GUI用にコメントアウト ★★★
            
            except Exception as e:
                    print(f"[重心スレッド] 処理中に予期せぬエラー: {e}")
                    pass
        
        time.sleep(0.001) 

    cap.release()
    print("[重心スレッド]: カメラを解放しました。")