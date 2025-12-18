import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160           # 底面での内側へのオフセット
ANGLE_LEFT_DEG = 50       # 左線の角度
ANGLE_RIGHT_DEG = 130     # 右線の角度

# 閾値の設定
THRESHOLD_CENTER = 15.0   # 中央進路の回避高さ
THRESHOLD_SIDE = 6.0      # 左右クローラの回避高さ（転倒防止）

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

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 座標の計算 ---
        y_top = int(h * 2 / 3)
        dy = h - y_top
        dx_left = int(dy / math.tan(math.radians(ANGLE_LEFT_DEG)))
        dx_right = int(dy / math.tan(math.radians(180 - ANGLE_RIGHT_DEG)))

        # 各頂点の定義
        pt_lb_outer = (0, h)                # 左下・外（青）
        pt_lt_outer = (w // 3, y_top)       # 左上・外（青）
        pt_rb_outer = (w, h)                # 右下・外（青）
        pt_rt_outer = (w * 2 // 3, y_top)   # 右上・外（青）

        pt_lb_inner = (OFFSET_PX, h)                # 左下・内（黄）
        pt_lt_inner = (OFFSET_PX + dx_left, y_top)  # 左上・内（黄）
        pt_rb_inner = (w - OFFSET_PX, h)            # 右下・内（黄）
        pt_rt_inner = (w - OFFSET_PX - dx_right, y_top) # 右上・内（黄）

        # --- 2. 領域（マスク）の作成 ---
        # 中央領域
        mask_center = np.zeros((h, w), dtype=np.uint8)
        pts_center = np.array([[pt_lb_inner, pt_lt_inner, pt_rt_inner, pt_rb_inner]], np.int32)
        cv2.fillPoly(mask_center, pts_center, 255)

        # 左クローラ領域（青線と黄線の間）
        mask_left = np.zeros((h, w), dtype=np.uint8)
        pts_left = np.array([[pt_lb_outer, pt_lt_outer, pt_lt_inner, pt_lb_inner]], np.int32)
        cv2.fillPoly(mask_left, pts_left, 255)

        # 右クローラ領域（黄線と青線の間）
        mask_right = np.zeros((h, w), dtype=np.uint8)
        pts_right = np.array([[pt_rb_inner, pt_rt_inner, pt_rt_outer, pt_rb_outer]], np.int32)
        cv2.fillPoly(mask_right, pts_right, 255)

        # --- 3. 高さ計算と判定 ---
        depth_cm = depth_image * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        def get_max_h(mask):
            valid = (mask > 0) & (depth_image > 0)
            return np.max(height_map[valid]) if np.any(valid) else 0

        max_h_c = get_max_h(mask_center)
        max_h_l = get_max_h(mask_left)
        max_h_r = get_max_h(mask_right)

        # --- 4. 警告表示と視覚化 ---
        # 中央判定
        if max_h_c > THRESHOLD_CENTER:
            cv2.putText(color_image, "FRONT OBSTACLE!", (w//2-100, 100), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 2)
        
        # 左判定
        if max_h_l > THRESHOLD_SIDE:
            cv2.putText(color_image, "LEFT TILT RISK!", (20, 150), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0,165,255), 2)

        # 右判定
        if max_h_r > THRESHOLD_SIDE:
            cv2.putText(color_image, "RIGHT TILT RISK!", (w-250, 150), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0,165,255), 2)

        # 数値の表示
        cv2.putText(color_image, f"C:{max_h_c:.1f} L:{max_h_l:.1f} R:{max_h_r:.1f} (cm)", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 線の描画
        cv2.polylines(color_image, [np.array([pt_lb_outer, pt_lt_outer, pt_rt_outer, pt_rb_outer], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, pt_lb_inner, pt_lt_inner, (0, 255, 255), 3)
        cv2.line(color_image, pt_rb_inner, pt_rt_inner, (0, 255, 255), 3)

        cv2.imshow('Obstacle Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()