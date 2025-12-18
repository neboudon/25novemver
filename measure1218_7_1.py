import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0
OFFSET_PX = 160
ANGLE_LEFT_DEG = 50
ANGLE_RIGHT_DEG = 130

# --- 閾値設定 ---
THRESHOLD_CENTER = 4.0
THRESHOLD_LEFT = 3.5
THRESHOLD_RIGHT = 2.0

# 1. パイプラインとアライメントの設定
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# 深度をカラーに合わせるためのアライメントオブジェクト
align_to = rs.stream.color
align = rs.align(align_to)

# 2. イントリンジック（内部パラメータ）の取得
# 深度をカラーに投影するため、カラーカメラのパラメータを使用します
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
intr = color_stream.get_intrinsics()
fy, cy = intr.fy, intr.ppy

try:
    while True:
        # フレームの取得
        frames = pipeline.wait_for_frames()
        
        # --- アライメント処理の実行 ---
        # これにより、深度マップの各ピクセルがカラー画像のピクセルと一致します
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # --- 深度データの取得とノイズ除去 ---
        depth_data = np.asanyarray(depth_frame.get_data())
        depth_data = cv2.medianBlur(depth_data, 5) 

        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 座標の計算 (カラー画像の解像度基準) ---
        y_top = int(h * 2 / 3)
        dy = h - y_top
        dx_left = int(dy / math.tan(math.radians(ANGLE_LEFT_DEG)))
        dx_right = int(dy / math.tan(math.radians(180 - ANGLE_RIGHT_DEG)))

        pts_in = [(OFFSET_PX, h), (OFFSET_PX + dx_left, y_top), 
                  (w - OFFSET_PX - dx_right, y_top), (w - OFFSET_PX, h)]
        pts_out = [(0, h), (w // 3, y_top), (w * 2 // 3, y_top), (w, h)]

        # --- 2. マスク作成 ---
        def create_mask(pts):
            m = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
            return m

        mask_c = create_mask(pts_in)
        mask_l = create_mask([pts_out[0], pts_out[1], pts_in[1], pts_in[0]])
        mask_r = create_mask([pts_in[3], pts_in[2], pts_out[2], pts_out[3]])

        # --- 3. 高さ計算 ---
        # depth_dataはカラーカメラの視点に変換されているため、
        # カラーの焦点距離(fy)と中心点(cy)を使って正しく高さを計算できます
        depth_cm = depth_data * 0.1 
        v_indices = np.indices((h, w))[0]
        height_map = CAMERA_HEIGHT_CM - ((v_indices - cy) * depth_cm / fy)

        def get_max_h(mask):
            # アライメントにより、端の方など深度データがない(0)場所ができるため、フィルタリング
            valid = (mask > 0) & (depth_data > 0)
            return np.max(height_map[valid]) if np.any(valid) else 0

        max_h_c = get_max_h(mask_c)
        max_h_l = get_max_h(mask_l)
        max_h_r = get_max_h(mask_r)

        # --- 4. 警告と描画 ---
        msg_color = (0, 0, 255)
        if max_h_c > THRESHOLD_CENTER:
            cv2.putText(color_image, "FRONT!", (w//2-50, 100), cv2.FONT_HERSHEY_DUPLEX, 1.2, msg_color, 3)
        if max_h_l > THRESHOLD_LEFT:
            cv2.putText(color_image, "L_TILT!", (20, 150), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 165, 255), 2)
        if max_h_r > THRESHOLD_RIGHT:
            cv2.putText(color_image, "R_TILT!", (w-200, 150), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 165, 255), 2)

        cv2.polylines(color_image, [np.array(pts_out, np.int32)], True, (255, 0, 0), 2)
        cv2.line(color_image, pts_in[0], pts_in[1], (0, 255, 255), 3)
        cv2.line(color_image, pts_in[3], pts_in[2], (0, 255, 255), 3)
        cv2.putText(color_image, f"C:{max_h_c:.1f} L:{max_h_l:.1f} R:{max_h_r:.1f}", (10, 30), 1, 1.5, (255, 255, 255), 2)

        cv2.imshow('Aligned Obstacle Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()