import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160
ANGLE_LEFT_DEG = 50
ANGLE_RIGHT_DEG = 130
THRESHOLD_ALL = 5.0  # すべての領域で5cm以上

# RealSenseの設定
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# アライメント設定
align = rs.align(rs.stream.color)
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fy, cy = intr.fy, intr.ppy

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        # 深度データの取得と基本ノイズ除去
        depth_data = np.asanyarray(depth_frame.get_data())
        depth_data = cv2.medianBlur(depth_data, 5) 
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 領域の座標計算 ---
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

        masks = {
            "CENTER": create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b]),
            "LEFT":   create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b]),
            "RIGHT":  create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])
        }

        # --- 2. 地面からの高さマップ計算 ---
        depth_cm = depth_data * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        # --- 3. 【新ロジック】上位5%の平均高さを算出する関数 ---
        def get_top_5_avg_h(mask):
            valid = (mask > 0) & (depth_data > 0)
            target_data = height_map[valid]
            if target_data.size > 0:
                # 95パーセンタイル（下から95%目）の値を境界とする
                threshold_val = np.percentile(target_data, 95)
                # その境界以上の値（上位5%）を抽出
                top_5_percent_values = target_data[target_data >= threshold_val]
                # その平均を返す
                return np.mean(top_5_percent_values)
            return 0

        # 各レーンの高さを取得
        results = { lane: get_top_5_avg_h(m) for lane, m in masks.items() }

        # --- 4. 警告表示 ---
        alert_y = 100
        for lane, val in results.items():
            if val >= THRESHOLD_ALL:
                # 中央は赤、左右はオレンジで表示
                color = (0, 0, 255) if lane == "CENTER" else (0, 165, 255)
                cv2.putText(color_image, f"OBSTACLE: {lane} LANE ({val:.1f}cm)", (20, alert_y), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.8, color, 2)
                alert_y += 40

        # 視覚化描画
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 3)
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 3)
        
        # 画面上部にリアルタイム数値を表示
        status_text = f"Top 5% Avg - L:{results['LEFT']:.1f} C:{results['CENTER']:.1f} R:{results['RIGHT']:.1f}"
        cv2.putText(color_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('Top 5% Average Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()