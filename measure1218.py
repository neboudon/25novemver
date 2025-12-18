import pyrealsense2 as rs
import numpy as np
import cv2

# --- 設定 ---
CAMERA_HEIGHT_CM = 21.0  # カメラの設置高さ

# RealSenseの設定
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# ストリーミング開始
profile = pipeline.start(config)

# カメラの内部パラメータを取得
intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fy = intr.fy
cy = intr.ppy

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 画像をnumpy配列に変換
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]

        # --- 1. 領域（ROI）の定義：下の対角線領域 ---
        # 頂点：左下、中央、右下
        points = np.array([[(0, h), (w // 2, h // 2), (w, h)]], dtype=np.int32)
        
        # マスクの作成
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, points, 255)

        # --- 2. 高さの計算 ---
        # 処理を軽くするため、マスク領域内のピクセルをサンプリング
        # (ここではデモ用に領域全体の高さを計算する考え方を示します)
        
        # 深度をcmに変換 (RealSenseのデフォルト単位は1mm = 0.001m)
        depth_cm = depth_image * 0.1 

        # 垂直ピクセル座標(v)のグリッド作成
        v_indices = np.indices((h, w))[0]
        
        # カメラ座標系でのY(下方向への距離)を計算
        # y_cam = (v - cy) * Z / fy
        y_cam = (v_indices - cy) * depth_cm / fy
        
        # 地面からの高さ = 設置高さ - カメラ座標のY
        height_from_ground = CAMERA_HEIGHT_CM - y_cam

        # マスク範囲内の高さデータのみ抽出（有効な深度のみ）
        valid_mask = (mask > 0) & (depth_image > 0)
        target_heights = height_from_ground[valid_mask]

        if target_heights.size > 0:
            avg_height = np.mean(target_heights)
            max_height = np.max(target_heights)
            # 画面に情報を表示
            cv2.putText(color_image, f"Avg Height: {avg_height:.1f}cm", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(color_image, f"Max Height: {max_height:.1f}cm", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # --- 3. 視覚化：境界線の描画 ---
        cv2.line(color_image, (0, h), (w // 2, h // 2), (255, 0, 0), 3)   # 左対角線
        cv2.line(color_image, (w, h), (w // 2, h // 2), (255, 0, 0), 3)   # 右対角線

        # 表示
        cv2.imshow('Obstacle Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()