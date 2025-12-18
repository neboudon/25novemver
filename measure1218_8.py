import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160           # 底面での内側へのオフセット
ANGLE_LEFT_DEG = 50       # 左線の角度
ANGLE_RIGHT_DEG = 130     # 右線の角度

# --- 閾値設定 (すべての領域で5cm以上に設定) ---
THRESHOLD_ALL = 5.0

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fy, cy = intr.fy, intr.ppy

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        # --- 深度データのノイズ除去 (高感度維持のため5x5フィルタ) ---
        depth_data = np.asanyarray(depth_frame.get_data())
        depth_data = cv2.medianBlur(depth_data, 5) 

        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 座標の計算 (領域の分割) ---
        y_top = int(h * 2 / 3)
        dy = h - y_top
        dx_left = int(dy / math.tan(math.radians(ANGLE_LEFT_DEG)))
        dx_right = int(dy / math.tan(math.radians(180 - ANGLE_RIGHT_DEG)))

        # 頂点定義
        p_li_b, p_li_t = (OFFSET_PX, h), (OFFSET_PX + dx_left, y_top)
        p_ri_b, p_ri_t = (w - OFFSET_PX, h), (w - OFFSET_PX - dx_right, y_top)
        p_lo_b, p_lo_t = (0, h), (w // 3, y_top)
        p_ro_b, p_ro_t = (w, h), (w * 2 // 3, y_top)

        # --- 2. マスク作成 ---
        def create_mask(pts):
            m = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
            return m

        mask_c = create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b])
        mask_l = create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b])
        mask_r = create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])

        # --- 3. 高さ計算 ---
        depth_cm = depth_data * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        def get_max_h(mask):
            valid = (mask > 0) & (depth_data > 0)
            return np.max(height_map[valid]) if np.any(valid) else 0

        max_h_c = get_max_h(mask_c)
        max_h_l = get_max_h(mask_l)
        max_h_r = get_max_h(mask_r)

        # --- 4. 警告表示 (どのレーンで検出したかを表示) ---
        alert_y = 100
        if max_h_l > THRESHOLD_ALL:
            cv2.putText(color_image, f"OBSTACLE: LEFT LANE ({max_h_l:.1f}cm)", (20, alert_y), 
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 165, 255), 2)
            alert_y += 40
            
        if max_h_c > THRESHOLD_ALL:
            cv2.putText(color_image, f"OBSTACLE: CENTER LANE ({max_h_c:.1f}cm)", (20, alert_y), 
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)
            alert_y += 40

        if max_h_r > THRESHOLD_ALL:
            cv2.putText(color_image, f"OBSTACLE: RIGHT LANE ({max_h_r:.1f}cm)", (20, alert_y), 
                        cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 165, 255), 2)

        # --- 描画と情報の表示 ---
        # 境界線
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 3)
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 3)

        # 現在の各エリアの最大値表示
        cv2.putText(color_image, f"Max H - L:{max_h_l:.1f} C:{max_h_c:.1f} R:{max_h_r:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Triple Lane Obstacle Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()