import pyrealsense2 as rs
import numpy as np
import cv2

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0  # カメラの設置高さ
OFFSET_PX = 30           # 内側へずらすピクセル数

# RealSenseの設定
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# ストリーミング開始
profile = pipeline.start(config)

# カメラの内部パラメータを取得
intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fy = intr.fy
cy = intr.ppy

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 画像をnumpy配列に変換
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 台形領域（ROI）の座標計算 ---
        y_top = int(h * 2 / 3)
        x_left_top = int(w / 3)
        x_right_top = int(w * 2 / 3)

        # 元の台形の頂点
        trapezoid_points = np.array([[(0, h), (x_left_top, y_top), (x_right_top, y_top), (w, h)]], dtype=np.int32)
        
        # マスクの作成
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, trapezoid_points, 255)

        # --- 2. 高さの計算 ---
        depth_cm = depth_image * 0.1 
        v_indices = np.indices((h, w))[0]
        y_cam = (v_indices - cy) * depth_cm / fy
        height_from_ground = CAMERA_HEIGHT_CM - y_cam

        valid_mask = (mask > 0) & (depth_image > 0)
        target_heights = height_from_ground[valid_mask]

        if target_heights.size > 0:
            max_height = np.max(target_heights)
            color = (0, 255, 0)
            if max_height > 15.0:
                color = (0, 0, 255)
            cv2.putText(color_image, f"Max: {max_height:.1f}cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # --- 3. 視覚化：線の描画 ---
        # A. 元の台形を描画（青色）
        cv2.polylines(color_image, trapezoid_points, isClosed=True, color=(255, 0, 0), thickness=2)

        # B. 左側の追加線（同じ傾きで30px内側）
        # 開始点: (30, h), 終了点: (x_left_top + 30, y_top)
        cv2.line(color_image, (OFFSET_PX, h), (x_left_top + OFFSET_PX, y_top), (0, 255, 255), 2)

        # C. 右側の追加線（同じ傾きで30px内側）
        # 開始点: (w - 30, h), 終了点: (x_right_top - 30, y_top)
        cv2.line(color_image, (w - OFFSET_PX, h), (x_right_top - OFFSET_PX, y_top), (0, 255, 255), 2)

        # 表示
        cv2.imshow('Trapezoid Path with Offset Lines', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()