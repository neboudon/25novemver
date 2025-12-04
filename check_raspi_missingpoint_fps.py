import cv2
import numpy as np
import time
import threading
import queue
import math

# --- 1. 設定・パラメータ ---
# カメラID (通常は0。複数のカメラがある場合は1, 2...と変更)
CAMERA_ID = 0

# カメラのキャプチャ設定（カメラ入力の場合、ここが重要です）
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
CAPTURE_FPS = 30

# 画像処理リサイズ設定
RESIZE_WIDTH = 240
MIN_NOISE_AREA = 45

#画像処理用のパラメータ
CANNY_THRESHOLD1 = 100
CANNY_THRESHOLD2 = 150
HOUGH_THRESHOLD = 35 
HOUGH_MIN_LINE_LENGTH = 35
HOUGH_MAX_LINE_GAP = 10
CLIP_LIMIT = 15.0
TILE_GRID_SIZE = (4, 4)


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
    
    
    # 計測用変数と準備
    frame_count = 0
    start_time = time.time()
    prev_time = start_time
    process_times = [] # 処理時間の履歴
    resized_frame = cv2.resize(first_frame, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
    height, width = resized_frame.shape[:2]
    image_center_x = width / 2
    all_process_times = []
    clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
    
    print("\n--- Start Camera Benchmarking ---")
    print("Press 'q' to stop.")

    try:
        while cap_thread.running():
            # 1. フレーム取得
            frame = cap_thread.read()
            if frame is None: continue

            # --- 計測開始 (処理単体) ---
            proc_start = time.perf_counter()

            # リサイズ & グレースケール
            latest_resized = cv2.resize(frame, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
            frame_gray = cv2.cvtColor(latest_resized, cv2.COLOR_BGR2GRAY)
            
            #clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
            adjusted = clahe.apply(frame_gray)
            blurred_again = cv2.GaussianBlur(adjusted, (7, 7), 0)
            edges = cv2.Canny(blurred_again, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
            
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edges, connectivity=8)
            cleaned_edges = np.zeros_like(edges)
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] > MIN_NOISE_AREA:
                    cleaned_edges[labels == i] = 255
            
            lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi/180, threshold=HOUGH_THRESHOLD, minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)
            
            diagonal_lines = []
            vp_x = width // 2
            steering_success = False 
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle_rad = math.atan2(y2 - y1, x2 - x1)
                    angle_deg = math.degrees(angle_rad)
                    abs_angle_deg = abs(angle_deg)

                    is_horizontal = (abs_angle_deg <= 10) or (abs_angle_deg >= 175)
                    is_vertical = (50 <= abs_angle_deg <= 130)
                    
                    if is_horizontal or is_vertical or x1 == x2:
                        continue
                    m = (y2 - y1) / (x2 - x1)
                    c = y1 - m * x1
                    diagonal_lines.append((m, c))
            
            intersection_points = []
            if len(diagonal_lines) >= 2:
                for i in range(len(diagonal_lines)):
                    for j in range(i + 1, len(diagonal_lines)):
                        m1, c1 = diagonal_lines[i]
                        m2, c2 = diagonal_lines[j]
                        if abs(m1 - m2) < 1e-5 or m1*m2 >0 : continue
                        x = (c2 - c1) / (m1 - m2)
                        y = m1 * x + c1
                        if -width < x < width * 2 and -height < y < height * 2:
                            intersection_points.append((x, y))
            if intersection_points:
                x_coords = [p[0] for p in intersection_points]
                vp_x = int(np.median(x_coords))
                steering_success = True 
            
            steering_difference = vp_x - image_center_x
            
            # --- 計測終了 (処理単体) ---
            proc_end = time.perf_counter()
            proc_time_ms = (proc_end - proc_start) * 1000
            #process_times.append(proc_time_ms)
            #if len(process_times) > 100: process_times.pop(0)
            all_process_times.append(proc_time_ms) # 全履歴に追加（捨てない）
            
            frame_count += 1
            
            # FPS計測とコンソール出力 (1秒ごと)
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                #avg_proc = sum(process_times)/len(process_times) if process_times else 0
                # リストの末尾100個だけを取り出して平均を計算
                recent_avg = sum(all_process_times[-100:]) / len(all_process_times[-100:]) if all_process_times else 0
                print(f"FPS: {fps:.2f} | Avg Proc Time: {recent_avg:.2f}ms | Steering Diff: {steering_difference:.2f} | Success: {steering_success}")
                frame_count = 0
                prev_time = curr_time
                
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Stopping...")
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