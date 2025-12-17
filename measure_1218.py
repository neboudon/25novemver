import cv2
import pyrealsense2 as rs
import numpy as np
import threading
import time

# --- カメラ管理クラス (継承) ---
class CameraStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.w, self.h = 640, 480
        self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, 30)
        profile = self.pipeline.start(self.config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.latest_color = None
        self.latest_depth = None
        self.is_running = True
        
    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        
    def update(self):
        while self.is_running:
            frames = self.pipeline.wait_for_frames()
            color_f = frames.get_color_frame()
            depth_f = frames.get_depth_frame()
            if not color_f or not depth_f: continue
            with self.lock:
                self.latest_color = np.asanyarray(color_f.get_data())
                self.latest_depth = np.asanyarray(depth_f.get_data())
                
    def get_latest(self):
        with self.lock:
            return self.latest_color, self.latest_depth

    def stop(self):
        self.is_running = False
        self.pipeline.stop()

def main():
    camera = CameraStream()
    camera.start()
    
    INTERVAL = 0.5 # 2Hz
    MAX_DISPLAY_DIST = 4.0 # ヒートマップの最大距離(4m)
    
    try:
        while True:
            start_time = time.time()
            color_img, depth_img = camera.get_latest()
            
            if color_img is None or depth_img is None:
                continue
            
            h, w = depth_img.shape
            # --- 1. 下半分を切り出し ---
            roi_y_start = h // 2
            depth_bottom = depth_img[roi_y_start:, :]
            color_bottom = color_img[roi_y_start:, :].copy()
            
            # --- 2. 全地点の距離を色で表現 (ヒートマップ) ---
            # 距離を0-255のスケールに変換 (MAX_DISPLAY_DISTを上限とする)
            depth_meters = depth_bottom * camera.depth_scale
            depth_display = np.clip(depth_meters / MAX_DISPLAY_DIST * 255, 0, 255).astype(np.uint8)
            
            # 色付け（JET: 青が遠く、赤が近い）
            depth_colormap = cv2.applyColorMap(255 - depth_display, cv2.COLORMAP_JET)
            
            # --- 3. 特定地点の距離数値を表示 (1/30間隔のグリッド) ---
            grid_step = 60 # 60ピクセルおきに数値を表示
            for y in range(0, depth_bottom.shape[0], grid_step):
                for x in range(0, depth_bottom.shape[1], grid_step):
                    dist = depth_meters[y, x]
                    if dist > 0:
                        text = f"{dist:.2f}"
                        # ヒートマップ上に数値を白字で表示
                        cv2.putText(depth_colormap, text, (x, y + 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 元のカラー画像に「計測中」の枠を表示
            cv2.rectangle(color_img, (0, roi_y_start), (w-1, h-1), (0, 255, 0), 2)
            
            # --- 表示 ---
            cv2.imshow("Original (Bottom Half ROI)", color_img)
            cv2.imshow("Bottom Half Distance Map (2Hz)", depth_colormap)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # 2Hz制御
            elapsed = time.time() - start_time
            wait_time = INTERVAL - elapsed
            if wait_time > 0:
                time.sleep(wait_time)

    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()