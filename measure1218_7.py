import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160
ANGLE_LEFT_DEG = 50
ANGLE_RIGHT_DEG = 130

# --- 閾値設定 (ユーザー様の環境に合わせた数値) ---
THRESHOLD_CENTER = 4.0
THRESHOLD_LEFT = 3.5
THRESHOLD_RIGHT = 2.0

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

        pt_lb_inner = (OFFSET_PX, h)
        pt_lt_inner = (OFFSET_PX + dx_left, y_top)
        pt_rb_inner = (w - OFFSET_PX, h)
        pt_rt_inner = (w - OFFSET_PX - dx_right, y_top)
        
        pt_lb_outer, pt_lt_outer = (0, h), (w // 3, y_top)
        pt_rb_outer, pt_rt_outer = (w, h), (w * 2 // 3, y_top)

        # --- 2. マスク作成 ---
        mask_center = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_center, [np.array([pt_lb_inner, pt_lt_inner, pt_rt_inner, pt_rb_inner])], 255)

        mask_left = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_left, [np.array([pt_lb_outer, pt_lt_outer, pt_lt_inner, pt_lb_inner])], 255)

        mask_right = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_right, [np.array([pt_rb_inner, pt_rt_inner, pt_rt_outer, pt_rb_outer])], 255)

        # --- 3. 高さ計算と【統計的フィルタリング】 ---
        depth_cm = depth_image * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        # 修正ポイント：最大値ではなく95パーセンタイルを取得
        def get_filtered_h(mask):
            valid = (mask > 0) & (depth_image > 0)
            target_data = height_map[valid]
            if target_data.size > 0:
                # 上位5%の異常値を無視して、確かな高さを採用
                return np.percentile(target_data, 95)
            return 0

        max_h_c = get_filtered_h(mask_center)
        max_h_l = get_filtered_h(mask_left)
        max_h_r = get_filtered_h(mask_right)

        # --- 4. 警告表示 ---
        if max_h_c > THRESHOLD_CENTER:
            cv2.putText(color_image, f"FRONT OBSTACLE! ({max_h_c:.1f})", (w//2-140, 100), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 2)
        if max_h_l > THRESHOLD_LEFT:
            cv2.putText(color_image, f"LEFT TILT! ({max_h_l:.1f})", (20, 150), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0,165,255), 2)
        if max_h_r > THRESHOLD_RIGHT:
            cv2.putText(color_image, f"RIGHT TILT! ({max_h_r:.1f})", (w-280, 150), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0,165,255), 2)

        # 数値と描画
        cv2.putText(color_image, f"C:{max_h_c:.1f} L:{max_h_l:.1f} R:{max_h_r:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.polylines(color_image, [np.array([pt_lb_outer, pt_lt_outer, pt_rt_outer, pt_rb_outer], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, pt_lb_inner, pt_lt_inner, (0, 255, 255), 3)
        cv2.line(color_image, pt_rb_inner, pt_rt_inner, (0, 255, 255), 3)

        cv2.imshow('Stable Detection (Percentile 95)', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()