import cv2
import pyrealsense2 as rs
import numpy as np
import threading
import time

# --- カメラ管理クラス (共通) ---
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
    
    INTERVAL = 0.5 # 2Hz (0.5秒間隔)
    MAX_DISPLAY_DIST = 5.0 # ヒートマップの最大距離(5m)。環境に合わせて変更してください。
    
    try:
        print("Starting Full-Screen Heatmap (2Hz)... Press 'q' to quit.")
        while True:
            start_time = time.time()
            color_img, depth_img = camera.get_latest()
            
            if color_img is None or depth_img is None:
                continue
            
            # --- 1. 全画面の距離計算 (ベクトル演算で高速化) ---
            depth_meters = depth_img * camera.depth_scale
            
            # 距離を0-255のスケールに変換 (0m=赤, MAX=青 にするために反転)
            # 0(無効値)を考慮しつつクリッピング
            depth_display = np.clip(depth_meters / MAX_DISPLAY_DIST * 255, 0, 255).astype(np.uint8)
            
            # --- 2. ヒートマップの生成 ---
            # JETカラーマップを適用 (青:遠い, 赤:近い)
            heatmap = cv2.applyColorMap(255 - depth_display, cv2.COLORMAP_JET)
            
            # データが取れていない場所（距離0）を黒にする
            heatmap[depth_img == 0] = [0, 0, 0]
            
            # --- 3. グリッド状に数値を表示 (視認性のため80ピクセル間隔) ---
            grid_step = 80 
            h, w = depth_img.shape
            for y in range(grid_step // 2, h, grid_step):
                for x in range(grid_step // 2, w, grid_step):
                    dist = depth_meters[y, x]
                    if dist > 0:
                        text = f"{dist:.1f}m"
                        # 背景を少し暗くして文字を見やすくする（オプション）
                        cv2.putText(heatmap, text, (x, y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            # --- 4. 表示 ---
            # 元画像とヒートマップを水平に連結して比較表示
            combined_view = np.hstack((color_img, heatmap))
            
            # 画面サイズが大きすぎる場合はリサイズ（表示用）
            display_scale = 0.8
            cv2.imshow("Full Screen Heatmap (Left: RGB, Right: Depth)", 
                       cv2.resize(combined_view, None, fx=display_scale, fy=display_scale))
            
            # 終了判定
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # 2Hz制御のためのウェイト
            elapsed = time.time() - start_time
            wait_time = INTERVAL - elapsed
            if wait_time > 0:
                time.sleep(wait_time)

    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()