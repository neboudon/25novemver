import cv2
import numpy as np
import math

#コマンド送信用の関数
def send_motor_command(ser,command):
    ser.write(command.encode('utf-8'))
    ser.flush()
    print(f"Sending command: {command}")
    
#画像の暗点重心を検出する関数
def process_cog(color_image,resize_h):
    # 重心検出処理の変数の定義
    STEERING_THRESHOLD = 5  # 重心差の閾値（ピクセル）
    RESIZE_WIDTH = 240
    
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
    
    # 重心位置に赤い円を描画
    cv2.circle(resized_frame, (center_x, height // 2), 5, (0, 0, 255), -1)
    # 中心線を描画（緑）
    cv2.line(resized_frame, (image_center_x, 0), (image_center_x, height), (0, 255, 0), 1)
    
    steering_command = "F\n"
    
    if abs(gravity_difference) > STEERING_THRESHOLD:
        if gravity_difference > 0:
            steering_command = f"R {gravity_difference:.2f}\n" 
        else:
            steering_command = f"L {abs(gravity_difference):.2f}\n"
    
    #return steering_command
    return steering_command, resized_frame


#画像の線から消失点を求める関数
def process_mis(color_image, resize_h, clahe):
    # 重心検出処理の変数の定義
    STEERING_THRESHOLD = 5  # 重心差の閾値（ピクセル）
    RESIZE_WIDTH = 240
    # Cannyエッジ検出の低閾値
    CANNY_THRESHOLD1 = 100
    # Cannyエッジ検出の高閾値
    CANNY_THRESHOLD2 = 150
    # ハフ変換の投票数の閾値
    HOUGH_THRESHOLD = 35
    # 検出する線の最小長
    HOUGH_MIN_LINE_LENGTH = 35
    # 線上の点と見なすための最大間隔
    HOUGH_MAX_LINE_GAP = 10
    # ノイズとみなす最小面積（ピクセル数）▼▼▼
    MIN_NOISE_AREA = 45
    
    #リサイズとグレースケール変換
    resized_frame = cv2.resize(color_image, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
    height, width = resized_frame.shape[:2]
    center_x = width // 2
    
    #前処理
    gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
    adjusted = clahe.apply(gray)
    blurred_again = cv2.GaussianBlur(adjusted, (7,7), 0)
    
    #Cannyでエッジ検出
    edges = cv2.Canny(blurred_again, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
    
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edges, connectivity=8)

    # ノイズ除去後のエッジ画像を格納するための黒い画像を用意
    cleaned_edges = np.zeros_like(edges)

    # 各連結成分をループ処理（ラベル0は背景なので1から始める）
    for i in range(1, num_labels):
        # 面積を取得
        area = stats[i, cv2.CC_STAT_AREA]
        # 面積が閾値より大きい場合のみ、その成分を新しい画像に描画
        if area > MIN_NOISE_AREA:
            cleaned_edges[labels == i] = 255
            
    #確率的ハフ変換
    lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi/180, threshold=HOUGH_THRESHOLD, minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)
    
    # 描画用のカラー画像を作成
    line_image = np.copy(resized_frame)
    
    diagonal_lines = []
     #消失点の計算
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            angle_rad = math.atan2(y2 - y1, x2 - x1)
            angle_deg = math.degrees(angle_rad)
            abs_angle_deg = abs(angle_deg)
            
            is_horizontal = (abs_angle_deg <= 10) or (abs_angle_deg >= 175)
            is_vertical = (50 <= abs_angle_deg <= 130)

            if is_horizontal or is_vertical:
                continue

            if x1 == x2: # 垂直線の場合
                continue
            else:
                m = (y2 - y1) / (x2 - x1)
                c = y1 - m * x1
            
            diagonal_lines.append((m, c))
            
            # 画面端まで線を延長して描画
            points = []
            if m != 0:
                x_at_y0 = -c / m
                if 0 <= x_at_y0 <= width: points.append((int(x_at_y0), 0))
                x_at_y_height = (height - c) / m
                if 0 <= x_at_y_height <= width: points.append((int(x_at_y_height), height))
            y_at_x0 = c
            if 0 <= y_at_x0 <= height: points.append((0, int(y_at_x0)))
            y_at_x_width = m * width + c
            if 0 <= y_at_x_width <= height: points.append((width, int(y_at_x_width)))
        
            if len(points) >= 2:
                cv2.line(line_image, points[0], points[1], (0, 255, 0), 2)
            
    
    intersection_points = []
    if len(diagonal_lines) >= 2:
        for i in range(len(diagonal_lines)):
            for j in range(i + 1, len(diagonal_lines)):
                m1, c1 = diagonal_lines[i]
                m2, c2 = diagonal_lines[j]

                if abs(m1 - m2) < 1e-5: 
                    continue
                
                if m1*m2>0:
                    continue
                
                x = (c2 - c1) / (m1 - m2)
                y = m1 * x + c1

                if -width < x < width * 2 and -height < y < height * 2:
                    intersection_points.append((x, y))
    
    steering_command = "F\n"
    
    if intersection_points:
        x_coords = [p[0] for p in intersection_points]
        vp_x = int(np.median(x_coords))
        cv2.circle(line_image, (vp_x, height//2), 10, (0, 0, 255), -1)
        
        if 0 <= vp_x < width:
            missing_difference = vp_x - center_x 
            if abs(missing_difference) > STEERING_THRESHOLD:
                if missing_difference > 0:
                    steering_command = f"R {missing_difference:.2f}\n" 
                else:
                    steering_command = f"L {abs(missing_difference):.2f}\n"
    
    cv2.line(line_image, (center_x, 0), (center_x, height), (0, 255, 0), 1)
    #return steering_command
    return steering_command, line_image

# --- 障害物検出用の関数 (measure_dist2_1_1.py より移植) ---
def process_obstacle_detection(color_image, depth_cm, ref_depth, lane_masks,base_center_x):
    DIFF_THRESHOLD_CM = 10.0
    MIN_AREA = 1350
    h, w = color_image.shape[:2]
    
    # 基準との差分計算
    diff = ref_depth - depth_cm
    obs_mask = np.where(diff > DIFF_THRESHOLD_CM, 255, 0).astype(np.uint8)
    obs_mask = cv2.morphologyEx(obs_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    
    contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_info = [] # 検出された情報を格納するリスト
    
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_AREA:
            continue
            
        x, y, bw, bh = cv2.boundingRect(cnt)
        obj_center_x = x + (bw / 2) # 障害物の中心x座標を追加
        diff_x = base_center_x - obj_center_x
        """
        detected_lanes = []
        blob_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
        
        for name, m in lane_masks.items():
            if np.any(cv2.bitwise_and(blob_mask, m) > 0):
                detected_lanes.append(name)
        """
        dist_val = np.nanmedian(depth_cm[y:y+bh, x:x+bw])
        detected_info.append({"dist": dist_val, "center_x": obj_center_x, "diff_of_center_x":diff_x})
        
        # 描画処理
        #label = f"{'/'.join(detected_lanes)}: {dist_val:.1f}cm"
        label = f"Dist: {dist_val:.1f}cm" # 距離のみ表示
        cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cv2.putText(color_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.line(color_image, (int(base_center_x), 0), (int(base_center_x), h), (255, 0, 0), 2)
        
    return color_image, detected_info