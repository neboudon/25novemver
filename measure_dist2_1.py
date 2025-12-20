import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 調整パラメータ (ここを変更してください) ---
# 1. 外側の境界線 (一番左右の線)
OUTER_OFFSET_PX = 50       # 底面の角をどれだけ内側に寄せるか (px)
ANGLE_OUTER_L_DEG = 60     # 左外線の角度 (度)
ANGLE_OUTER_R_DEG = 120    # 右外線の角度 (度)

# 2. 内側の境界線 (レーン間の区切り線)
INNER_OFFSET_PX = 170      # 底面の開始位置 (px)
ANGLE_INNER_L_DEG = 45     # 左内線の角度 (度)
ANGLE_INNER_R_DEG = 135    # 右内線の角度 (度)

# 3. 共通設定
Y_TOP_RATIO = 2/3          # 台形の上辺の高さ (画面下からの割合)
DIFF_THRESHOLD_CM = 10.0   # 障害物検知のしきい値 (cm)
CALIB_FRAMES = 5           # キャリブレーションフレーム数
CAMERA_HEIGHT_CM = 21.0    # カメラの設置高度

# --- システム設定 ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# --- 領域計算 ---
tmp_frames = pipeline.wait_for_frames()
color_tmp = np.asanyarray(align.process(tmp_frames).get_color_frame().get_data())
h, w = color_tmp.shape[:2]

y_top = int(h * Y_TOP_RATIO)
dy = h - y_top

# 各線のx方向の伸び(dx)を計算
dx_inner_l = int(dy / math.tan(math.radians(ANGLE_INNER_L_DEG)))
dx_inner_r = int(dy / math.tan(math.radians(180 - ANGLE_INNER_R_DEG)))
dx_outer_l = int(dy / math.tan(math.radians(ANGLE_OUTER_L_DEG)))
dx_outer_r = int(dy / math.tan(math.radians(180 - ANGLE_OUTER_R_DEG)))

# 座標の定義 (b: bottom, t: top)
p_lo_b, p_lo_t = (OUTER_OFFSET_PX, h), (OUTER_OFFSET_PX + dx_outer_l, y_top)
p_ro_b, p_ro_t = (w - OUTER_OFFSET_PX, h), (w - OUTER_OFFSET_PX - dx_outer_r, y_top)
p_li_b, p_li_t = (INNER_OFFSET_PX, h), (INNER_OFFSET_PX + dx_inner_l, y_top)
p_ri_b, p_ri_t = (w - INNER_OFFSET_PX, h), (w - INNER_OFFSET_PX - dx_inner_r, y_top)

def create_mask(pts):
    m = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
    return m

lane_masks = {
    "LEFT":   create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b]),
    "CENTER": create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b]),
    "RIGHT":  create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])
}

total_lane_mask = cv2.bitwise_or(lane_masks["LEFT"], lane_masks["CENTER"])
total_lane_mask = cv2.bitwise_or(total_lane_mask, lane_masks["RIGHT"])

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
            cv2.imshow('Detection', color_image)
            if calib_counter == CALIB_FRAMES: ref_depth = accum_depth / CALIB_FRAMES
            cv2.waitKey(1)
            continue

        # 差分による検出
        diff = ref_depth - depth_cm
        obs_mask = np.where(diff > DIFF_THRESHOLD_CM, 255, 0).astype(np.uint8)
        obs_mask = cv2.morphologyEx(obs_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # 塊の描画とレーン判定
        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < 500: continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            
            detected_lanes = [name for name, m in lane_masks.items() 
                              if np.any(cv2.bitwise_and(create_mask(cnt), m) > 0)]
            
            dist_val = np.nanmedian(depth_cm[y:y+bh, x:x+bw])
            cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.putText(color_image, f"{'/'.join(detected_lanes)}: {dist_val:.1f}cm", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 境界線の描画
        cv2.line(color_image, p_lo_b, p_lo_t, (255, 0, 0), 2) # 左外
        cv2.line(color_image, p_ro_b, p_ro_t, (255, 0, 0), 2) # 右外
        cv2.line(color_image, p_li_b, p_li_t, (0, 255, 255), 2) # 左内
        cv2.line(color_image, p_ri_b, p_ri_t, (0, 255, 255), 2) # 右内

        cv2.imshow('Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()