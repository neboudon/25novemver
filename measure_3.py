import pyrealsense2 as rs
import numpy as np
import cv2
import time

# --- 0. 標準入力からROIの座標を受け取る ---
print("測定したい領域（ROI）の情報を入力してください。")
# 入力値のバリデーション（数値変換エラー）を考慮
try:
    roi_x = int(input("  ROIの左上のX座標: "))
    roi_y = int(input("  ROIの左上のY座標: "))
    roi_w = int(input("  ROIの幅 (width): "))
    roi_h = int(input("  ROIの高さ (height): "))
except ValueError:
    print("エラー: 数値を入力してください。")
    exit()

# --- 1. パイプラインの設定 ---
pipeline = rs.pipeline()
config = rs.config()

# 解像度やフォーマットを指定
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# --- 2. ストリーミング開始 ---
print("\nストリーミングを開始します... (ウィンドウで 'q' を押すと終了)")
profile = pipeline.start(config)

# デプススケールを取得（距離計算に必要）
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# センサーが安定するまで少し待つ
time.sleep(2)

# alignオブジェクトを作成 (デプスとカラーの位置合わせのため)
align_to = rs.stream.color
align = rs.align(align_to)

try:
    # --- 3. ループ処理でリアルタイム表示 ---
    while True:
        # --- フレームの取得とアライメント ---
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # --- NumPy配列への変換 ---
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # --- 4. ROI座標の計算とクリッピング ---
        # 入力された座標が画像の範囲を超える場合に備えて調整（クリッピング）
        h, w, _ = color_image.shape
        roi_x1 = max(0, roi_x)
        roi_y1 = max(0, roi_y)
        # roi_x2 (右下X) = roi_x1 + 幅
        roi_x2 = min(w, roi_x1 + roi_w)
        # roi_y2 (右下Y) = roi_y1 + 高さ
        roi_y2 = min(h, roi_y1 + roi_h)

        # --- 5. ROI領域の距離計算 ---
        # 深度画像からROI領域を切り出す
        depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]

        # 深度値が0のピクセル（測定不能）を除外
        non_zero_depth = depth_roi[depth_roi > 0]

        avg_distance_meters = 0.0
        if non_zero_depth.size > 0:
            # 平均を計算し、デプススケールを掛けてメートルに変換
            avg_distance_meters = np.mean(non_zero_depth) * depth_scale

        # --- 6. 結果の可視化 (OpenCV) ---
        
        # カラー画像に入力されたROIの四角を描画
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 2)

        # 計算した平均距離をテキストで表示
        dist_text = f"Distance: {avg_distance_meters:.2f} m"
        cv2.putText(color_image, dist_text, (roi_x1, roi_y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 画像を表示
        cv2.imshow("RealSense - ROI Distance", color_image)

        # --- 7. 標準出力への出力 ---
        # \r を使うことで、ターミナルの同じ行に距離を上書き表示
        print(f"\r選択領域 ({roi_x},{roi_y} 幅{roi_w}x高さ{roi_h}) の平均距離: {avg_distance_meters:.2f} m", end="")

        # 'q' キーで終了
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

finally:
    # --- 8. ストリーミング停止 ---
    pipeline.stop()
    cv2.destroyAllWindows()
    # 最後のprint出力が残るように改行
    print("\nストリーミングを停止しました。")