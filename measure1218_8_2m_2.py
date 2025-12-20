import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 170
ANGLE_LEFT_DEG = 45
ANGLE_RIGHT_DEG = 135
THRESHOLD_SIDE = 5.0   
THRESHOLD_CENT = 13.0  
PERCENTILE_THRESHOLD = 95 # 上位5%

# RealSenseの設定
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)

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

        # --- 3. 障害物検出とノイズ除去・描画 ---
        results = {}
        
        for lane_name, mask in masks.items():
            current_threshold = THRESHOLD_CENT if lane_name == "CENTER" else THRESHOLD_SIDE
            
            valid = (mask > 0) & (depth_data > 0)
            target_heights = height_map[valid]
            
            if target_heights.size > 0:
                # 統計的代表値（表示用）
                thresh_5 = np.percentile(target_heights, 95)
                results[lane_name] = np.mean(target_heights[target_heights >= thresh_5])

                # 上位5%かつ設定しきい値以上の領域をマスク化
                obs_mask = np.zeros((h, w), dtype=np.uint8)
                obs_mask[(mask > 0) & (height_map >= max(thresh_5, current_threshold))] = 255
                
                # --- 輪郭抽出と面積によるノイズ除去 ---
                contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for cnt in contours:
                    # 【追加】面積が50ピクセル未満ならノイズとしてスキップ
                    if cv2.contourArea(cnt) < 50:
                        continue
                    
                    # 輪郭を赤色で描画
                    cv2.drawContours(color_image, [cnt], -1, (0, 0, 255), 2)
                    
                    # オプション：外接矩形を描画して距離を表示
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    dist = np.median(depth_cm[y:y+bh, x:x+bw])
                    cv2.putText(color_image, f"{dist:.1f}cm", (x, y - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                results[lane_name] = 0

        # --- 4. 警告表示と視覚化 ---
        alert_y = 100
        for lane, val in results.items():
            thresh = THRESHOLD_CENT if lane == "CENTER" else THRESHOLD_SIDE
            if val >= thresh:
                color = (0, 0, 255) if lane == "CENTER" else (0, 165, 255)
                cv2.putText(color_image, f"OBSTACLE: {lane} LANE ({val:.1f}cm)", (20, alert_y), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.8, color, 2)
                alert_y += 40

        # レーン境界線の描画
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 3)
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 3)
        
        status_text = f"Top 5% Avg - L:{results['LEFT']:.1f} C:{results['CENTER']:.1f} R:{results['RIGHT']:.1f}"
        cv2.putText(color_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('Top 5% Contour with Noise Filter', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()