import pyrealsense2 as rs
import numpy as np
import cv2
import math

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0  # 床面からカメラレンズ中心までの高さ
CAMERA_PITCH_DEG = 0.0   # カメラの仰角（下向きならマイナス、水平なら0）
OFFSET_PX = 160          # 画面下部のトリミング
# 閾値（ユーザー定義）
THRESHOLD_CRAWLER = 5.0  # クローラが回避すべき高さ (5cm)
THRESHOLD_BODY = 15.0     # ロボット本体が回避すべき高さ (15cm)

# --- RealSense初期設定 ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# パフォーマンス向上のためのフィルタ
decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 2) # 解像度を半分にして高速化

profile = pipeline.start(config)
intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fy, cy = intr.fy, intr.ppy

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        # フィルタ適用
        depth_frame = decimation.process(depth_frame)
        
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        # numpy配列に変換
        depth_data = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        h, w = depth_data.shape # フィルタ後サイズ (320x240等)
        orig_h, orig_w = color_image.shape[:2]

        # --- 1. 高さマップの計算 (ベクトル演算で高速化) ---
        # 深度をcmに変換
        depth_cm = depth_data * 0.1
        # 画面上の各ピクセルの垂直方向インデックス
        v_indices = np.arange(h).reshape(-1, 1)
        # 簡易3D投影による床面からの高さ計算
        # H = Cam_H - (dist * cos(theta)) ※カメラが水平の場合
        # ここではユーザーの式をベースに一般化
        height_map = CAMERA_HEIGHT_CM - ((v_indices - (cy/2)) * depth_cm / (fy/2))

        # --- 2. 領域（ROI）の定義 ---
        # 座標は現在のdepth解像度に合わせてスケーリング
        y_limit = int(h * 2 / 3) # フレーム上部1/3をカット
        
        # マスク作成用
        mask_body = np.zeros((h, w), dtype=np.uint8)
        mask_crawler_l = np.zeros((h, w), dtype=np.uint8)
        mask_crawler_r = np.zeros((h, w), dtype=np.uint8)

        # 領域の頂点 (三角形・台形) を定義
        # 中央（本体）: 15cm閾値
        pts_body = np.array([
            [w//4, h], [w//3, y_limit], [2*w//3, y_limit], [3*w//4, h]
        ], np.int32)
        cv2.fillPoly(mask_body, [pts_body], 255)

        # 左側（クローラ）: 5cm閾値
        pts_l = np.array([
            [0, h], [w//3, y_limit], [w//4, h]
        ], np.int32)
        cv2.fillPoly(mask_crawler_l, [pts_l], 255)

        # 右側（クローラ）: 5cm閾値
        pts_r = np.array([
            [3*w//4, h], [2*w//3, y_limit], [w, h]
        ], np.int32)
        cv2.fillPoly(mask_crawler_r, [pts_r], 255)

        # --- 3. 2段階閾値による判定 ---
        def check_obstacle(m, threshold, min_pixels=10):
            # マスク範囲内で、かつ有効な深度データがあり、閾値を超えているピクセルを抽出
            valid_mask = (m > 0) & (depth_data > 0)
            obstacles = height_map[valid_mask] > threshold
            return np.sum(obstacles) > min_pixels

        # 判定実行
        danger_body = check_obstacle(mask_body, THRESHOLD_BODY)
        danger_l = check_obstacle(mask_crawler_l, THRESHOLD_CRAWLER)
        danger_r = check_obstacle(mask_crawler_r, THRESHOLD_CRAWLER)

        # --- 4. 結果の描画 ---
        # 可視化用にマスクをカラー画像サイズにリサイズ
        display_img = cv2.resize(color_image, (orig_w, orig_h))
        
        if danger_body:
            cv2.putText(display_img, "BODY COLLISION!", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)
        if danger_l or danger_r:
            side = "BOTH" if (danger_l and danger_r) else ("LEFT" if danger_l else "RIGHT")
            cv2.putText(display_img, f"CRAWLER BLOCK: {side}", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,165,255), 2)

        # 検出エリアの可視化
        overlay = display_img.copy()
        cv2.fillPoly(overlay, [(pts_body * (orig_w/w)).astype(int)], (0, 255, 0)) # 本体パス
        cv2.fillPoly(overlay, [(pts_l * (orig_w/w)).astype(int)], (255, 255, 0)) # 左クローラ
        cv2.fillPoly(overlay, [(pts_r * (orig_w/w)).astype(int)], (255, 255, 0)) # 右クローラ
        cv2.addWeighted(overlay, 0.3, display_img, 0.7, 0, display_img)

        cv2.imshow('Dual Threshold Detection', display_img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
finally:
    pipeline.stop()