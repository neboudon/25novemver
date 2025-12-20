import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 170
ANGLE_LEFT_DEG = 45
ANGLE_RIGHT_DEG = 135
THRESHOLD_ALL = 5.0  # 障害物とみなす高さのしきい値(cm)
PERCENTILE_THRESHOLD = 97  # 上位3% (100 - 3)

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
        depth_data_blur = cv2.medianBlur(depth_data, 5) 
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
        depth_cm = depth_data_blur * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        # --- 3. 障害物検出と描画のロジック ---
        results = {}
        
        for lane_name, mask in masks.items():
            valid = (mask > 0) & (depth_data > 0)
            target_heights = height_map[valid]
            
            if target_heights.size > 0:
                # 1. 上位5%平均（既存ロジック）
                thresh_5 = np.percentile(target_heights, 95)
                avg_5 = np.mean(target_heights[target_heights >= thresh_5])
                results[lane_name] = avg_5

                # 2. 【新規】上位3%かつ5cm以上の領域を赤枠で囲む
                thresh_3 = np.percentile(target_heights, PERCENTILE_THRESHOLD)
                # 判定用マスク：現在のレーン内 かつ 高さが上位3%の境界以上 かつ 5cm以上
                obs_limit = max(thresh_3, THRESHOLD_ALL)
                specific_obs_mask = np.zeros((h, w), dtype=np.uint8)
                specific_obs_mask[(mask > 0) & (height_map >= obs_limit)] = 255
                
                # 輪郭を見つけて囲む
                contours, _ = cv2.findContours(specific_obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    if cv2.contourArea(cnt) < 50: continue # 小さすぎるノイズを除外
                    
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    # 障害物までの距離（深度）を取得（矩形中心付近の深度平均）
                    roi_depth = depth_cm[y:y+bh, x:x+bw]
                    valid_depths = roi_depth[roi_depth > 0]
                    if valid_depths.size > 0:
                        dist = np.median(valid_depths)
                        
                        # 赤枠と距離の描画
                        cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
                        cv2.putText(color_image, f"{dist:.1f}cm", (x, y - 5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                results[lane_name] = 0

        # --- 4. 既存の警告表示と視覚化 ---
        alert_y = 100
        for lane, val in results.items():
            if val >= THRESHOLD_ALL:
                color = (0, 0, 255) if lane == "CENTER" else (0, 165, 255)
                cv2.putText(color_image, f"OBSTACLE: {lane} LANE ({val:.1f}cm)", (20, alert_y), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.8, color, 2)
                alert_y += 40

        # 境界線の描画
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 3)
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 3)
        
        status_text = f"Top 5% Avg - L:{results['LEFT']:.1f} C:{results['CENTER']:.1f} R:{results['RIGHT']:.1f}"
        cv2.putText(color_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('Top 3% Detection & Distance', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()