#
# ファイル名: camera_processing.py
# 役割: RealSenseカメラの制御と、指定された領域の距離計算を行うモジュール
#

import pyrealsense2 as rs
import numpy as np
import cv2
import time

class WallDetector:
    """
    RealSenseカメラを管理し、壁までの距離を検出するクラス
    """
    def __init__(self, width=424, height=240, fps= 15):
        print(f"カメラモジュールを初期化中... (解像度: {width}x{height})")
        self.W = width
        self.H = height

        # --- パイプラインの設定 ---
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, fps)

        # --- ストリーミング開始 ---
        profile = self.pipeline.start(config)

        # デプススケールを取得
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # アライメントオブジェクト
        self.align = rs.align(rs.stream.color)

        # --- ROIの定義 (固定) ---
        # 画面の上下100ピクセル、幅30ピクセルを「壁」検出領域とする
        self.roi_h = 100  # ROIの高さ
        self.roi_w = 30   # ROIの幅
        self.roi_y1 = (self.H // 2) - (self.roi_h // 2) # Y座標 (上)70
        self.roi_y2 = self.roi_y1 + self.roi_h          # Y座標 (下)170

        print("カメラの準備ができました。")
        time.sleep(2) # センサー安定待機

    def _calculate_distance_in_roi(self, depth_image, side):
        """
        [依頼された関数]
        深度画像と方向('left'/'right')を受け取り、
        その方向のROIの平均距離を計算する
        """
        
        # side に基づいてROIのX座標を決定 #roi_x1からroi_x2までの範囲の距離を計算
        if side == 'right':
            # 右壁をチェック (画面の右端)
            roi_x1 = self.W - self.roi_w
            roi_x2 = self.W
        elif side == 'left':
            # 左壁をチェック (画面の左端)
            roi_x1 = 0
            roi_x2 = self.roi_w
        else:
            # 不正なsideが指定された
            return 0.0, (0,0,0,0) # 距離0と空の座標を返す

        # --- 5. ROI領域の距離計算 ---
        depth_roi = depth_image[self.roi_y1:self.roi_y2, roi_x1:roi_x2]
        non_zero_depth = depth_roi[depth_roi > 0] # 0 (測定不能) を除外

        avg_distance_meters = 0.0
        if non_zero_depth.size > 0:
            avg_distance_meters = np.mean(non_zero_depth) * self.depth_scale

        # 可視化用にROIの座標も返す
        return avg_distance_meters, (roi_x1, self.roi_y1, roi_x2, self.roi_y2)

    def get_frame_and_distance(self, side_to_check):
        """
        最新のフレームを取得し、指定された側の壁距離を計算する
        """
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None, None

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # [関数呼び出し] 距離を計算
            distance, roi_coords = self._calculate_distance_in_roi(depth_image, side_to_check)

            # --- 6. 結果の可視化 ---
            # 制御ロジックとは別だが、デバッグ用にROIを描画する
            (x1, y1, x2, y2) = roi_coords
            if distance > 0:
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                dist_text = f"Side: {side_to_check} / Dist: {distance:.2f} m"
            else:
                # 距離が0（測定不能）の場合は赤く表示
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                dist_text = f"Side: {side_to_check} / NO DATA"
                
            cv2.putText(color_image, dist_text, (x1 - 100, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            return distance , color_image

        except Exception as e:
            print(f"エラー: {e}")
            return None, None

    def stop(self):
        """
        ストリーミングを停止する
        """
        print("カメラを停止します。")
        self.pipeline.stop()

# このファイルが直接実行された場合のテスト用
if __name__ == "__main__":
    detector = WallDetector()
    try:
        while True:
            # 'right' 側をテスト
            #distance = detector.get_frame_and_distance(side_to_check='left')
            distance, color_image = detector.get_frame_and_distance(side_to_check='right')
            
            if color_image is None:
                continue

            cv2.imshow("Test Camera View", color_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
    finally:
        detector.stop()
        cv2.destroyAllWindows()