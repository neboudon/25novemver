import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160
ANGLE_LEFT_DEG = 50
ANGLE_RIGHT_DEG = 130
THRESHOLD_ALL = 5.0

# 1. RealSenseの設定とアライメントオブジェクトの作成
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# カラーフレームに深度フレームを合わせるためのアライメントオブジェクト
align_to = rs.stream.color
align = rs.align(align_to)

# アライメント後はカラーカメラの内部パラメータを使用する
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fy, cy = intr.fy, intr.ppy

try:
    while True:
        # フレーム待ち
        frames = pipeline.wait_for_frames()
        
        # 2. アライメントの実行 (深度フレームをカラーフレームの位置に合わせる)
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not aligned_depth_frame or not color_frame:
            continue

        # 3. データの変換とノイズ除去
        depth_data = np.asanyarray(aligned_depth_frame.get_data())
        depth_data = cv2.medianBlur(depth_data, 5) # 点ノイズ除去
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 4. 座標と領域の計算 ---
        y_top = int(h * 2 / 3)
        dy = h - y_top
        dx_l = int(dy / math.tan(math.radians(ANGLE_LEFT_DEG)))
        dx_r = int(dy / math.tan(math.radians(180 - ANGLE_RIGHT_DEG)))

        p_li_b, p_li_t = (OFFSET_PX, h), (OFFSET_PX + dx_l, y_top)
        p_ri_b, p_ri_t = (w - OFFSET_PX, h), (w - OFFSET_PX - dx_r, y_top)
        p_lo_b, p_lo_t = (0, h), (w // 3, y_top)
        p_ro_b, p_ro_t = (w, h), (w * 2 // 3, y_top)

        def create_mask(pts):
            m = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
            return m

        mask_c = create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b])
        mask_l = create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b])
        mask_r = create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])

        # --- 5. 地面からの高さ計算 ---
        # $Height = CAMERA\_HEIGHT - \frac{(v - cy) \cdot Z}{fy}$
        depth_cm = depth_data * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        def get_max_h(mask):
            valid = (mask > 0) & (depth_data > 0)
            return np.max(height_map[valid]) if np.any(valid) else 0

        max_h_c = get_max_h(mask_c)
        max_h_l = get_max_h(mask_l)
        max_h_r = get_max_h(mask_r)

        # --- 6. 警告表示と描画 ---
        alert_y = 100
        for label, val in zip(["LEFT", "CENTER", "RIGHT"], [max_h_l, max_h_c, max_h_r]):
            if val > THRESHOLD_ALL:
                color = (0, 0, 255) if label == "CENTER" else (0, 165, 255)
                cv2.putText(color_image, f"OBSTACLE: {label} LANE ({val:.1f}cm)", (20, alert_y), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, color, 2)
                alert_y += 40

        # ガイド線の描画
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 3)
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 3)

        cv2.putText(color_image, f"Aligned Mode - L:{max_h_l:.1f} C:{max_h_c:.1f} R:{max_h_r:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('Aligned Obstacle Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()