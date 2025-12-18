import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_INNER_PX = 160   # 内側に160ピクセルずらす
ANGLE_L_DEG = 80        # 左側の追加線の角度（度）
ANGLE_R_DEG = 100       # 右側の追加線の角度（度）

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
        y_bottom = h
        y_top = int(h * 2 / 3)
        delta_y = y_bottom - y_top # 高さの差（正の値）

        # 元の台形の頂点
        x_left_bottom_orig, x_right_bottom_orig = 0, w
        x_left_top_orig, x_right_top_orig = int(w / 3), int(w * 2 / 3)

        # --- 2. 追加線の座標計算（角度指定） ---
        # 左の線: (OFFSET_INNER_PX, y_bottom) からスタート
        # x_top = x_bottom + (delta_y / tan(angle))
        # ※画像座標系(yが下向き)での計算
        rad_l = math.radians(ANGLE_L_DEG)
        x_left_bottom_new = OFFSET_INNER_PX
        x_left_top_new = int(x_left_bottom_new + (delta_y / math.tan(rad_l)))

        # 右の線: (w - OFFSET_INNER_PX, y_bottom) からスタート
        rad_r = math.radians(ANGLE_R_DEG)
        x_right_bottom_new = w - OFFSET_INNER_PX
        x_right_top_new = int(x_right_bottom_new + (delta_y / math.tan(rad_r)))

        # --- 3. 視覚化：描画 ---
        # A. 元の台形（青色・細線）
        orig_pts = np.array([[(0, h), (x_left_top_orig, y_top), (x_right_top_orig, y_top), (w, h)]], np.int32)
        cv2.polylines(color_image, orig_pts, True, (255, 0, 0), 1)

        # B. 追加された急な線（黄色・太線）
        # 左の追加線
        cv2.line(color_image, (x_left_bottom_new, y_bottom), (x_left_top_new, y_top), (0, 255, 255), 2)
        # 右の追加線
        cv2.line(color_image, (x_right_bottom_new, y_bottom), (x_right_top_new, y_top), (0, 255, 255), 2)
        
        # 上底の線（幅を広げないよう、追加線の頂点間を結ぶ）
        cv2.line(color_image, (x_left_top_new, y_top), (x_right_top_new, y_top), (0, 255, 255), 2)

        # --- 4. 高さ計算（この追加領域内を対象にする場合） ---
        # 必要に応じて追加線に囲まれた領域のマスク処理をここで行います

        cv2.imshow('Steep Angle ROI', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()