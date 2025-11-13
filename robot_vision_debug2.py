# ファイル名: robot_vision_thread_headless1112.py
# (main_control_thread_serial2_1112.py と同じフォルダに保存してください)

import cv2
import numpy as np
import math
import time
import os # ★★★ ファイル/ディレクトリ操作のためにインポート ★★★

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
    TARGET_FPS = 5  # 毎秒5回
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
    【統合スレッド・デバッグ保存機能付き】
    'latest_frame' が更新されたら、それを使って処理を実行する。
    1. 操舵（消失点）検出を試みる。
    2. 失敗した場合のみ、同じフレームで重心検出を実行する。
    3. 処理結果を描画し、2秒に1回ファイルに保存する。
    """
    
    # --- 操舵用パラメータ ---
    CANNY_THRESHOLD1 = 100
    CANNY_THRESHOLD2 = 150
    HOUGH_THRESHOLD = 35 
    HOUGH_MIN_LINE_LENGTH = 35
    HOUGH_MAX_LINE_GAP = 10
    CLIP_LIMIT = 15.0
    TILE_GRID_SIZE = (4, 4)
    
    # --- デバッグ保存用設定 ---
    DEBUG_SAVE_INTERVAL = 2.0 # 画像を保存する間隔（秒）
    DEBUG_SAVE_DIR = "debug_images" # 保存先ディレクトリ
    last_save_time = time.time()

    # デバッグ用ディレクトリを作成
    if not os.path.exists(DEBUG_SAVE_DIR):
        os.makedirs(DEBUG_SAVE_DIR)
        print(f"[画像処理スレッド]: '{DEBUG_SAVE_DIR}' ディレクトリを作成しました。")

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
            
            # 元の画像(resized_frame)ではなく、描画用のコピー(debug_frame)を作成
            debug_frame = resized_frame.copy()
            
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
                    
                    # ★変更点: 元々ここにあった「検出した全線を描画する処理」を削除
                    
                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    angle_deg = math.degrees(angle_rad)
                    abs_angle_deg = abs(angle_deg)

                    is_horizontal = (abs_angle_deg <= 10) or (abs_angle_deg >= 175)
                    is_vertical = (50 <= abs_angle_deg <= 130)
                    
                    # フィルタリング: 水平でも垂直でもない線で、x1とx2が同じでない場合
                    if is_horizontal or is_vertical or x1 == x2:
                        continue # この線は無視する
                        
                    # --- 条件に適合した線のみ、以下の処理が実行される ---
                    m = (y2 - y1) / (x2 - x1)
                    c = y1 - m * x1
                    diagonal_lines.append((m, c))

                    # ★変更点: 条件を満たした線を延長して描画
                    # 画面の上端 (y=0) と下端 (y=height) のx座標を計算
                    y1_ext = 0
                    x1_ext = int((y1_ext - c) / m)
                    y2_ext = height
                    x2_ext = int((y2_ext - c) / m)

                    # 延長した線をデバッグ画像に描画 (緑色)
                    cv2.line(debug_frame, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 255, 0), 1)

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
                # --- 操舵が失敗した場合 ---
                gray_frame_gravity = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

                inverted_array = 255 - gray_frame_gravity
                total_weight = np.sum(inverted_array)
                center_x = width / 2 
                if total_weight > 0:
                    x_coords = np.arange(width)
                    center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

                gravity_difference = center_x - image_center_x
                
                with lock:
                    shared_state['steering_success'] = False
                    shared_state['gravity_value'] = gravity_difference
                    shared_state['steering_value'] = 0.0 
                
                # 重心（フォールバック）の情報をデバッグ画像に描画
                cv2.circle(debug_frame, (int(center_x), height // 2), 5, (0, 255, 255), -1) # 黄色の円
                cv2.putText(debug_frame, "MODE: GRAVITY (Fallback)", (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            else:
                # --- 操舵が成功した場合 ---
                with lock:
                    shared_state['steering_success'] = True
                    shared_state['steering_value'] = steering_difference
                
                # 消失点（操舵）の情報をデバッグ画像に描画
                cv2.circle(debug_frame, (vp_x, height // 2), 5, (0, 0, 255), -1) # 赤色の円
                cv2.putText(debug_frame, "MODE: STEERING", (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # 常に画像中央の線を描画 (青色)
            cv2.line(debug_frame, (int(image_center_x), 0), (int(image_center_x), height), (255, 0, 0), 1)

            # === 4. デバッグ画像のファイル保存 (間引き処理) ===
            current_time = time.time()
            if (current_time - last_save_time) > DEBUG_SAVE_INTERVAL:
                last_save_time = current_time
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(DEBUG_SAVE_DIR, f"debug_{timestamp}.jpg")
                
                cv2.imwrite(filename, debug_frame)
                
                # printは \n を先頭につけて、メインスレッドの行上書きを邪魔しないようにする
                print(f"\n[画像処理スレッド]: デバッグ画像を保存: {filename}")
                    
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
    # print("[壁検出スレッド]: 起動しましたが、このスクリプトでは使用されません。")
    pass