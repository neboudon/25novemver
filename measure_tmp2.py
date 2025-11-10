import pyrealsense2 as rs
import numpy as np
import cv2
import time

# パイプラインの作成
pipeline = rs.pipeline()
config = rs.config()

# Depth + Color ストリーム設定
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# パイプライン開始
profile = pipeline.start(config)

# カメラ起動待機（ウォームアップ時間）
time.sleep(2.0)

try:
    while True:
        # フレーム待機（最大5秒）
        frames = pipeline.wait_for_frames(timeout_ms=5000)

        # DepthとColorの取得
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            print("Frame not ready, skipping...")
            continue

        # numpy変換
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 表示
        cv2.imshow('RealSense Depth', cv2.convertScaleAbs(depth_image, alpha=0.03))
        cv2.imshow('RealSense RGB', color_image)

        # 終了判定
        if cv2.waitKey(1) == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
