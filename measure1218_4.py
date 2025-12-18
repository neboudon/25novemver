import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160           # 160ピクセル内側へ
ANGLE_LEFT_DEG = 60       # 左線の角度（度数法）
ANGLE_RIGHT_DEG = 120     # 右線の角度（度数法）

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

        # --- 1. 座標計算 ---
        y_top = int(h * 2 / 3)
        dy = h - y_top # 垂直方向の距離 (160px)

        # 三角関数で上底のx座標を計算 (内側に追い込む)
        # dx = dy / tan(theta)
        dx_left = int(dy / math.tan(math.radians(ANGLE_LEFT_DEG)))
        dx_right = int(dy / math.tan(math.radians(180 - ANGLE_RIGHT_DEG)))

        # 追加線の座標
        new_left_start = (OFFSET_PX, h)
        new_left_end = (OFFSET_PX + dx_left, y_top)
        
        new_right_start = (w - OFFSET_PX, h)
        new_right_end = (w - OFFSET_PX - dx_right, y_top)

        # 元の台形の座標（比較用・青線）
        trap_pts = np.array([[(0, h), (w//3, y_top), (w*2//3, y_top), (w, h)]], np.int32)

        # --- 2. 高さ計算（メインパス領域：青の台形内） ---
        depth_cm = depth_image * 0.1 
        v_indices = np.indices((h, w))[0]
        height_from_ground = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, trap_pts, 255)
        valid_mask = (mask > 0) & (depth_image > 0)
        
        if np.any(valid_mask):
            max_h = np.max(height_from_ground[valid_mask])
            cv2.putText(color_image, f"Max Height: {max_h:.1f}cm", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if max_h < 15 else (0, 0, 255), 2)

        # --- 3. 視覚化 ---
        # 元の台形（青）
        cv2.polylines(color_image, trap_pts, True, (255, 0, 0), 2)
        
        # 追加の急な線（黄色）
        cv2.line(color_image, new_left_start, new_left_end, (0, 255, 255), 3)
        cv2.line(color_image, new_right_start, new_right_end, (0, 255, 255), 3)

        cv2.imshow('Robot Vision', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()