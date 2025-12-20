import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 170
ANGLE_LEFT_DEG = 45
ANGLE_RIGHT_DEG = 135
DIFF_THRESHOLD_CM = 10.0  
CALIB_FRAMES = 5          

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# --- 1. レーン境界マスクの事前定義 ---
# (ループの外で一度だけ計算することで、この範囲外の計算を排除します)
tmp_frames = pipeline.wait_for_frames() # 形状取得用
color_tmp = np.asanyarray(align.process(tmp_frames).get_color_frame().get_data())
h, w = color_tmp.shape[:2]

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

# 全レーンを合わせた「計測対象エリア」のマスク
total_lane_mask = cv2.bitwise_or(lane_masks["LEFT"], lane_masks["CENTER"])
total_lane_mask = cv2.bitwise_or(total_lane_mask, lane_masks["RIGHT"])

# 変数初期化
calib_counter = 0
accum_depth = None
ref_depth = None

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        # 深度取得と「範囲外のカット」
        raw_depth = np.asanyarray(depth_frame.get_data()).astype(float)
        
        # 【重要】レーン範囲外のピクセルは計算対象から除外(NaNにする)
        raw_depth[total_lane_mask == 0] = np.nan
        raw_depth[raw_depth == 0] = np.nan
        depth_cm = raw_depth * 0.1
        
        color_image = np.asanyarray(color_frame.get_data())

        # --- 2. 基準マスク作成（レーン内のみ） ---
        if calib_counter < CALIB_FRAMES:
            if accum_depth is None:
                accum_depth = np.zeros((h, w), dtype=float)
            
            # 範囲内の有効な距離だけを累積
            accum_depth = np.nansum([accum_depth, depth_cm], axis=0)
            calib_counter += 1
            
            # 視覚的に「計測中」を表示
            cv2.fillPoly(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], (0, 50, 0))
            cv2.putText(color_image, f"CALIBRATING LANE AREA... {calib_counter}", 
                        (50, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('Restricted Detection', color_image)
            if calib_counter == CALIB_FRAMES:
                ref_depth = accum_depth / CALIB_FRAMES
            cv2.waitKey(1)
            continue

        # --- 3. 障害物抽出（レーン内のみ） ---
        # 基準(床)との差分。レーン外は既にNaNなので計算されない。
        diff = ref_depth - depth_cm
        obs_mask = np.where(diff > DIFF_THRESHOLD_CM, 255, 0).astype(np.uint8)
        
        # ノイズ除去
        obs_mask = cv2.morphologyEx(obs_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # --- 4. 塊検出と描画 ---
        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 500: continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            
            # どのレーンに重なっているか判定
            detected_lanes = []
            blob_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
            
            for name, m in lane_masks.items():
                if np.any(cv2.bitwise_and(blob_mask, m) > 0):
                    detected_lanes.append(name)
            
            dist_val = np.nanmedian(depth_cm[y:y+bh, x:x+bw])
            label = f"{'/'.join(detected_lanes)}: {dist_val:.1f}cm"
            
            cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.putText(color_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 境界線とエリア外の減光処理（どこを測っているか分かりやすくするため）
        overlay = color_image.copy()
        cv2.fillPoly(overlay, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], (100, 100, 100))
        color_image = cv2.addWeighted(overlay, 0.3, color_image, 0.7, 0)
        
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 255, 0), 2)
        cv2.imshow('Restricted Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()