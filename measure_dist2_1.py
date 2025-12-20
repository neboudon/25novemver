import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
Y_TOP_RATIO = 2/3      # 画面上部のどこまでを検出対象にするか
DIFF_THRESHOLD_CM = 10.0  
CALIB_FRAMES = 5          

# --- 4本の線のパラメータ設定 ---
# 角度(DEG)は水平方向を基準。dx = dy / tan(angle) で計算
# OFFSETは「左端から（Left）」または「右端から（Right）」の距離

# 1. 左外境界 (Left Outer)
OFF_L_OUT = 0          # 左端からのオフセット
ANG_L_OUT = 60         # 角度（急にするほど垂直に近い）

# 2. 左内境界 (Left Inner)
OFF_L_IN  = 170        # 左端からのオフセット
ANG_L_IN  = 45         # 角度

# 3. 右内境界 (Right Inner)
OFF_R_IN  = 170        # 右端からのオフセット
ANG_R_IN  = 45         # 角度

# 4. 右外境界 (Right Outer)
OFF_R_OUT = 0          # 右端からのオフセット
ANG_R_OUT = 60         # 角度

# --- パイプライン準備 ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# --- 1. レーン境界マスクの動的計算 ---
tmp_frames = pipeline.wait_for_frames()
color_tmp = np.asanyarray(align.process(tmp_frames).get_color_frame().get_data())
h, w = color_tmp.shape[:2]
y_top = int(h * Y_TOP_RATIO)
dy = h - y_top

def get_top_x(bottom_x, angle_deg, direction="left"):
    # dy / dx = tan(theta) => dx = dy / tan(theta)
    dx = int(dy / math.tan(math.radians(angle_deg)))
    return bottom_x + dx if direction == "left" else bottom_x - dx

# 各ラインの頂点計算
# 左外 (Left Outer)
p_lo_b = (OFF_L_OUT, h)
p_lo_t = (get_top_x(OFF_L_OUT, ANG_L_OUT, "left"), y_top)

# 左内 (Left Inner)
p_li_b = (OFF_L_IN, h)
p_li_t = (get_top_x(OFF_L_IN, ANG_L_IN, "left"), y_top)

# 右内 (Right Inner)
p_ri_b = (w - OFF_R_IN, h)
p_ri_t = (get_top_x(w - OFF_R_IN, ANG_R_IN, "right"), y_top)

# 右外 (Right Outer)
p_ro_b = (w - OFF_R_OUT, h)
p_ro_t = (get_top_x(w - OFF_R_OUT, ANG_R_OUT, "right"), y_top)

def create_mask(pts):
    m = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
    return m

lane_masks = {
    "LEFT":   create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b]),
    "CENTER": create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b]),
    "RIGHT":  create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])
}

# 全レーンの統合マスク（計測範囲全体）
total_lane_mask = cv2.bitwise_or(lane_masks["LEFT"], lane_masks["CENTER"])
total_lane_mask = cv2.bitwise_or(total_lane_mask, lane_masks["RIGHT"])

# --- 以下、メインループ処理 (変更なし) ---
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

        raw_depth = np.asanyarray(depth_frame.get_data()).astype(float)
        raw_depth[total_lane_mask == 0] = np.nan
        raw_depth[raw_depth == 0] = np.nan
        depth_cm = raw_depth * 0.1
        color_image = np.asanyarray(color_frame.get_data())

        if calib_counter < CALIB_FRAMES:
            if accum_depth is None: accum_depth = np.zeros((h, w), dtype=float)
            accum_depth = np.nansum([accum_depth, depth_cm], axis=0)
            calib_counter += 1
            cv2.putText(color_image, f"CALIBRATING... {calib_counter}", (50, h//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('Restricted Detection', color_image)
            if calib_counter == CALIB_FRAMES: ref_depth = accum_depth / CALIB_FRAMES
            cv2.waitKey(1)
            continue

        diff = ref_depth - depth_cm
        obs_mask = np.where(diff > DIFF_THRESHOLD_CM, 255, 0).astype(np.uint8)
        obs_mask = cv2.morphologyEx(obs_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 500: continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            detected_lanes = []
            blob_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
            for name, m in lane_masks.items():
                if np.any(cv2.bitwise_and(blob_mask, m) > 0): detected_lanes.append(name)
            dist_val = np.nanmedian(depth_cm[y:y+bh, x:x+bw])
            label = f"{'/'.join(detected_lanes)}: {dist_val:.1f}cm"
            cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.putText(color_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 描画処理：計測エリアの可視化
        overlay = color_image.copy()
        cv2.fillPoly(overlay, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], (100, 100, 100))
        color_image = cv2.addWeighted(overlay, 0.3, color_image, 0.7, 0)
        cv2.polylines(color_image, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 255, 0), 2)
        
        cv2.imshow('Restricted Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()