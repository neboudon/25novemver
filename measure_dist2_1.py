import pyrealsense2 as rs
import numpy as np
import cv2
import math

# ==========================================
# --- 調整パラメータ ---
# ==========================================
# 【左側の線】
L_OUT_OFFSET = 40      # 一番左の線の底面位置 (px)
L_OUT_ANGLE  = 65      # 一番左の線の傾き (度)
L_IN_OFFSET  = 180     # 左から2番目の線の底面位置 (px)
L_IN_ANGLE   = 50      # 左から2番目の線の傾き (度)

# 【右側の線】
R_IN_OFFSET  = 180     # 右から2番目の線の底面位置 (px)
R_IN_ANGLE   = 130     # 右から2番目の線の傾き (度)
R_OUT_OFFSET = 40      # 一番右の線の底面位置 (px)
R_OUT_ANGLE  = 115     # 一番右の線の傾き (度)

# 【共通設定】
CAMERA_HEIGHT_CM = 21.0
DIFF_THRESHOLD_CM = 10.0   # 障害物検知のしきい値
CALIB_FRAMES = 5           # 背景学習フレーム数
Y_TOP_RATIO = 2/3          # 監視エリアの上端高さ (画面下からの割合)

# ==========================================

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# 座標計算の初期化
tmp_frames = pipeline.wait_for_frames()
color_tmp = np.asanyarray(align.process(tmp_frames).get_color_frame().get_data())
h, w = color_tmp.shape[:2]

y_top = int(h * Y_TOP_RATIO)
dy = h - y_top

# 各線のx方向の伸び(dx)を計算
def get_dx(angle):
    # 90度付近でのエラー回避と度数法からラジアンへの変換
    rad = math.radians(angle)
    return int(dy / math.tan(rad)) if math.sin(rad) != 0 else 0

dx_lo = get_dx(L_OUT_ANGLE)
dx_li = get_dx(L_IN_ANGLE)
dx_ri = get_dx(180 - R_IN_ANGLE)
dx_ro = get_dx(180 - R_OUT_ANGLE)

# 4本の線の座標定義 (b: bottom, t: top)
p_lo_b, p_lo_t = (L_OUT_OFFSET, h), (L_OUT_OFFSET + dx_lo, y_top)
p_li_b, p_li_t = (L_IN_OFFSET, h),  (L_IN_OFFSET + dx_li, y_top)
p_ri_b, p_ri_t = (w - R_IN_OFFSET, h), (w - R_IN_OFFSET - dx_ri, y_top)
p_ro_b, p_ro_t = (w - R_OUT_OFFSET, h), (w - R_OUT_OFFSET - dx_ro, y_top)

def create_mask(pts):
    m = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
    return m

# 各レーンのマスク作成
lane_masks = {
    "LEFT":   create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b]),
    "CENTER": create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b]),
    "RIGHT":  create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b])
}

# 全エリアの合算マスク
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

        # 深度取得
        raw_depth = np.asanyarray(depth_frame.get_data()).astype(float)
        # マスク外と欠損値をNaNに
        raw_depth[total_lane_mask == 0] = np.nan
        raw_depth[raw_depth == 0] = np.nan
        depth_cm = raw_depth * 0.1
        color_image = np.asanyarray(color_frame.get_data())

        # --- 背景キャリブレーション ---
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

        # --- 障害物検出 ---
        diff = ref_depth - depth_cm
        obs_mask = np.where(diff > DIFF_THRESHOLD_CM, 255, 0).astype(np.uint8)
        obs_mask = cv2.morphologyEx(obs_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < 500: continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            
            # レーン判定
            blob_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(blob_mask, [cnt], -1, 255, -1)
            detected = [name for name, m in lane_masks.items() if np.any(cv2.bitwise_and(blob_mask, m) > 0)]
            
            dist_val = np.nanmedian(depth_cm[y:y+bh, x:x+bw])
            cv2.rectangle(color_image, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.putText(color_image, f"{'/'.join(detected)}: {dist_val:.1f}cm", (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # --- 視覚化 ---
        # 境界線の描画
        lines = [(p_lo_b, p_lo_t), (p_li_b, p_li_t), (p_ri_b, p_ri_t), (p_ro_b, p_ro_t)]
        for p1, p2 in lines:
            cv2.line(color_image, p1, p2, (0, 255, 255), 2)

        # エリア外を少し暗くする
        overlay = color_image.copy()
        cv2.fillPoly(overlay, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], (120, 120, 120))
        color_image = cv2.addWeighted(overlay, 0.2, color_image, 0.8, 0)

        cv2.imshow('Restricted Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()