import cv2
import numpy as np
import math
import time
import threading
import serial
import queue

# カメラID (通常は0。複数のカメラがある場合は1, 2...と変更)
CAMERA_ID = 0

# カメラのキャプチャ設定（カメラ入力の場合、ここが重要です）
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
CAPTURE_FPS = 30

#robot_vision_debug2からのパラメータ
RESIZE_WIDTH = 240
MIN_NOISE_AREA = 45 

# 表示設定
SHOW_WINDOW = False     # 計測中は False 推奨（描画負荷を排除するため）
SKIP_RENDERING = False   # Trueならimshowを完全にスキップ

    
# --- 3. マルチスレッド撮影クラス（カメラ専用） ---
class CameraCaptureThread:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        
        # カメラの設定（解像度とFPSを指定して安定化）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        
        # 実際の設定値を確認
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Camera Initialized: {self.width}x{self.height} @ {self.fps}FPS")

        self.q = queue.Queue(maxsize=1) # 最新フレームのみ保持
        self.stopped = False
        
        # スレッド開始
        self.t = threading.Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    def _reader(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                # カメラからの取得失敗（切断など）
                print("Camera read failed.")
                self.stopped = True
                break
            
            # ドロップフレーム処理：キューが一杯なら古いものを捨てて最新を入れる
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        try:
            return self.q.get(timeout=1) # タイムアウト付きで取得
        except queue.Empty:
            return None

    def running(self):
        return not self.stopped and self.cap.isOpened()

    def stop(self):
        self.stopped = True
        self.t.join()
        self.cap.release()

# --- 4. メイン計測処理 ---
def main():
    print(f"Attempting to open camera ID: {CAMERA_ID}")
    
    cap_thread = CameraCaptureThread(CAMERA_ID)
    time.sleep(2.0) # カメラの露出安定待ち

    # 初期フレーム取得
    first_frame = cap_thread.read()
    if first_frame is None:
        print("Failed to get first frame from camera.")
        cap_thread.stop()
        return

    # サイズ計算
    orig_h, orig_w = first_frame.shape[:2]
    aspect_ratio = orig_h / orig_w
    resize_h = int(RESIZE_WIDTH * aspect_ratio)
    
    #計測用変数
    frame_count = 0
    start_time = time.time()
    prev_time = start_time
    process_times = []
    all_process_times = []
    
    print("\n--- Start Camera Benchmarking ---")
    print("Press 'q' to stop.")
    
    try:
        while cap_thread.running():
            #フレーム取得
            frame = cap_thread.read()
            if frame is None:
                continue
            
            #計測開始
            proc_start = time.perf_counter()
            
            #リサイズと前処理
            resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
            height, width = resized_frame.shape[:2]
            gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
            
            #暗点重心の検出
            inverted_array = 255 - gray_frame
            total_weight = np.sum(inverted_array)
            image_center_x = width // 2
            if total_weight > 0:
                x_coords = np.arange(width)
                center_x = np.sum(x_coords * np.sum(inverted_array, axis=0)) / total_weight

            gravity_difference = center_x - image_center_x
            
            
            
            #計測終了
            proc_end = time.perf_counter()
            process_time_ms = (proc_end - proc_start)*1000
            all_process_times.append(process_time_ms)
            frame_count += 1
            
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                recent_avg = sum(all_process_times[-100:]) / len(all_process_times[-100:]) if all_process_times else 0
                print(f"FPS: {fps:.2f} | Avg Proc Time: {recent_avg:.2f}ms | Dark Point X Diff: {gravity_difference:.2f}")
                frame_count = 0
                prev_time = curr_time
    
    except KeyboardInterrupt:
        print("Benchmarking stopped by user.")
    finally:
        cap_thread.stop()
        cv2.destroyAllWindows()
        if all_process_times:
            print("\n" + "="*40)
            print(" FINAL BENCHMARK REPORT")
            print("="*40)
            print("--- Per Frame Processing Time ---")
            # 全データの表示ループ
            for i, t in enumerate(all_process_times):
                print(f"Frame {i+1:04d}: {t:.3f} ms")
            
            # 統計の計算
            total_avg = sum(all_process_times) / len(all_process_times)
            max_time = max(all_process_times)
            min_time = min(all_process_times)
            
            print("-" * 40)
            print(f"Total Frames Processed : {len(all_process_times)}")
            print(f"Average Processing Time: {total_avg:.3f} ms")
            print(f"Max Processing Time    : {max_time:.3f} ms")
            print(f"Min Processing Time    : {min_time:.3f} ms")
            print("="*40)
        else:
            print("\nNo frames processed.")

if __name__ == '__main__':
    main()