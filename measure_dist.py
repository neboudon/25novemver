import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 170
ANGLE_LEFT_DEG = 45
ANGLE_RIGHT_DEG = 135
DIFF_THRESHOLD_CM = 10.0  # 基準より何cm手前に来たら障害物とするか
CALIB_FRAMES = 5          # キャリブレーションに使用するフレーム数

# RealSenseの設定
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)

# 変数初期化
calib_counter = 0
accum_depth = None
ref_depth = None  # 基準となる距離マップ

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        depth_data = np.asanyarray(depth_frame.get_data()).astype(float)
        # 0（欠損値）を大きな値に置き換えて計算に影響させない工夫
        depth_data[depth_data == 0] = np.nan
        depth_cm = depth_data * 0.1 
        
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 基準マスク（背景）の作成（最初の5回） ---
        if calib_counter < CALIB_FRAMES:
            if accum_depth is None:
                accum_depth = np.zeros((h, w), dtype=float)
            
            # ナンを無視して累積（後で平均をとるため）
            accum_depth = np.nansum([accum_depth, depth_cm], axis=0)
            calib_counter += 1
            
            cv2.putText(color_image, f"Calibrating... {calib_counter}/{CALIB_FRAMES}", 
                        (200, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Detection', color_image)
            if calib_counter == CALIB_FRAMES:
                ref_depth = accum_depth / CALIB_FRAMES
                print("Calibration Complete.")
            cv2.waitKey(1)
            continue

        # --- 2. レーンマスクの作成（既存ロジック） ---
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

        lane_masks = {
            "LEFT":   create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b]),
            "CENTER": create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b]),
            "RIGHT":  create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])
        }

        # --- 3. 障害物ピクセルの抽出 ---
        # 基準距離より DIFF_THRESHOLD_CM 以上手前にあるものを 255 とする
        # (ref_depth - depth_cm) が正なら手前にある
        diff = ref_depth - depth_cm
        obs_mask = np.where(diff > DIFF_THRESHOLD_CM, 255, 0).astype(np.uint8)
        
        # ノイズ除去（クロージング処理）
        kernel = np.ones((5, 5), np.uint8)
        obs_mask = cv2.morphologyEx(obs_mask, cv2.MORPH_OPEN, kernel)

        # --- 4. 塊（コントゥア）の検出とレーン判定 ---
        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500: continue # 小さなノイズは無視

            x, y, bw, bh = cv2.boundingRect(cnt)
            
            # どのレーンに属しているか判定 (塊のマスクとレーンマスクの論理積)
            blob_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
            
            detected_lanes = []
            for lane_name, l_mask in lane_masks.items():
                # 塊の領域とレーンの重なりを確認
                overlap = cv2.bitwise_and(blob_mask, l_mask)
                if np.any(overlap > 0):
                    detected_lanes.append(lane_name)
            
            # 表示
            lane_str = ",".join(detected_lanes)
            dist_val = np.nanmedian(depth_cm[y:y+bh, x:x+bw])
            
            cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            label = f"{lane_str} {dist_val:.1f}cm"
            cv2.putText(color_image, label, (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 補助線の描画
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 0, 0), 1)
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 2)
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 2)

        cv2.imshow('Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()