import cv2
import pyrealsense2 as rs
import numpy as np
import threading
import time

# --- カメラ管理クラス (ご提示のコードをベースに作成) ---
class CameraStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 解像度とフレームレートの設定
        self.w, self.h = 640, 480
        self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, 30)
        
        profile = self.pipeline.start(self.config)
        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.is_running = True
        
    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        
    def update(self):
        while self.is_running:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            with self.lock:
                self.latest_color_frame = np.asanyarray(color_frame.get_data())
                self.latest_depth_frame = np.asanyarray(depth_frame.get_data())
                
    def get_latest(self):
        with self.lock:
            return self.latest_color_frame, self.latest_depth_frame

    def stop(self):
        self.is_running = False
        self.pipeline.stop()

# --- グラフ描画用関数 ---
def draw_depth_graph(distances, width, height=200, max_dist=5.0):
    """
    距離データをグラフ化する関数
    distances: 1次元の距離配列(m)
    max_dist: グラフの最大値（メートル）
    """
    graph_img = np.zeros((height, width, 3), dtype=np.uint8)
    num_points = len(distances)
    
    for i in range(1, num_points):
        # 前の点と現在の点の座標を計算 (y座標は距離に比例)
        x1 = int((i - 1) * (width / num_points))
        x2 = int(i * (width / num_points))
        
        # 距離をピクセル座標に変換（上限をmax_distに設定）
        d1 = min(distances[i-1], max_dist)
        d2 = min(distances[i], max_dist)
        
        y1 = int(height - (d1 / max_dist * height))
        y2 = int(height - (d2 / max_dist * height))
        
        cv2.line(graph_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # 目盛りの描画 (1m刻み)
    for m in range(int(max_dist) + 1):
        y = int(height - (m / max_dist * height))
        cv2.putText(graph_img, f"{m}m", (5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.line(graph_img, (0, y), (width, y), (50, 50, 50), 1)

    return graph_img

def main():
    camera = CameraStream()
    camera.start()
    
    print("Initializing camera...")
    time.sleep(2) # 安定待ち
    
    # 周期管理用 (2Hz = 0.5s)
    INTERVAL = 0.5 
    
    try:
        while True:
            start_time = time.time()
            
            # 1. 画像取得
            color_img, depth_img = camera.get_latest()
            if color_img is None or depth_img is None:
                continue
            
            h, w = depth_img.shape
            
            # 2. 短冊領域（ROI）の計算
            # 下から1/3の地点： h * 2/3
            center_y = int(h * 2 / 3)
            y_start = max(0, center_y - 20)
            y_end = min(h, center_y + 20)
            
            # 短冊領域を切り出して、横方向の平均距離を計算
            # 縦方向に平均をとることで1次元配列(幅サイズ)にする
            roi_depth = depth_img[y_start:y_end, :]
            # 0(無効値)を除外して平均をとるために、マスクを使用
            mask = (roi_depth > 0)
            horizontal_avg = np.zeros(w)
            for x in range(w):
                valid_pixels = roi_depth[:, x][mask[:, x]]
                if len(valid_pixels) > 0:
                    horizontal_avg[x] = np.mean(valid_pixels) * camera.depth_scale
                else:
                    horizontal_avg[x] = 0 # データなし
            
            # 3. 可視化処理
            # 元画像に計測範囲を表示
            display_img = color_img.copy()
            cv2.rectangle(display_img, (0, y_start), (w, y_end), (255, 0, 0), 2)
            cv2.putText(display_img, "Measurement ROI", (10, y_start - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # グラフの作成
            graph_img = draw_depth_graph(horizontal_avg, w)
            
            # 表示
            cv2.imshow("RealSense ROI View", display_img)
            cv2.imshow("Horizontal Distance Graph (2Hz)", graph_img)
            
            # 終了処理
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # 2Hzのための待機時間計算
            elapsed = time.time() - start_time
            sleep_time = INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # デバッグ用：中央の距離を表示
            print(f"Center Depth: {horizontal_avg[w//2]:.2f}m")

    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()