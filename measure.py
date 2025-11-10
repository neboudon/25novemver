import pyrealsense2 as rs
import time

# --- パイプラインの設定 ---
pipeline = rs.pipeline()
config = rs.config()

# デプスストリームとカラーストリームを有効化
# 一般的な解像度 (640x480) を指定
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# --- ストリーミング開始 ---
print("ストリーミングを開始します...")
profile = pipeline.start(config)

# センサーが安定するまで少し待つ
time.sleep(2)

try:
    # --- フレームの取得 ---
    # alignオブジェクトを作成 (デプスとカラーの位置合わせのため)
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 複数フレーム取得して安定させる
    for _ in range(5):
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

    # --- デプスフレームから距離情報を取得 ---
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        print("フレームが取得できませんでした。")
        exit()

    # --- 特定のピクセルの距離を取得 ---
    # 画面の中央のピクセル座標を指定
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    target_x = int(width / 2)
    target_y = int(height / 2)

    # get_distance(x, y) で (x, y) の距離 [メートル] を取得
    distance = depth_frame.get_distance(target_x, target_y)

    print(f"画面中央 ({target_x}, {target_y}) までの距離: {distance:.2f} メートル")


finally:
    # --- ストリーミング停止 ---
    pipeline.stop()
    print("ストリーミングを停止しました。")