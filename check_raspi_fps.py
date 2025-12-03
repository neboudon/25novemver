import cv2
import numpy as np
import time
import threading
import queue

# --- 1. 設定・パラメータ ---
# カメラID (通常は0。複数のカメラがある場合は1, 2...と変更)
CAMERA_ID = 0

# カメラのキャプチャ設定（カメラ入力の場合、ここが重要です）
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
CAPTURE_FPS = 30

# 画像処理リサイズ設定
RESIZE_WIDTH = 360

# 表示設定
SHOW_WINDOW = False     # 計測中は False 推奨（描画負荷を排除するため）
SKIP_RENDERING = False   # Trueならimshowを完全にスキップ

# --- パラメータ（元のコードと同じ） ---
TRACK_MAX_LEN = 50
RE_DETECT_INTERVAL = 10
MIN_TRACKS = 40
NEW_POINT_MIN_DIST = 15
TRAJECTORY_MIN_DY = 20.0 
TRAJECTORY_DRIFT_RATIO = 0.5
TRAJECTORY_MIN_POINTS = 5
CLUSTER_MIN_TRACKS = 3
CLUSTER_GRID_CELL_SIZE = 40
DETECTION_TTL = 15
FALL_MIN_DY = 10.0
MOVE_X_MAX = 10.0
GROUP_RADIUS = 40.0
MIN_GROUP_SIZE = 2

feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=10, blockSize=7)
lk_params = dict(winSize=(10, 10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# --- 2. 判定用関数群 ---
def judge_falling(track):
    start_point = track[0]; end_point = track[-1]
    start_x, start_y = start_point; end_x, end_y = end_point
    dy = end_y - start_y
    dx = end_x - start_x
    if dy < FALL_MIN_DY: return False
    if abs(dx) > MOVE_X_MAX: return False
    return True

def get_track_center(track):
    points = np.array(track)
    center = np.mean(points, axis=0)
    return center

def find_dense_falling_tracks(falling_candidates):
    if len(falling_candidates) <= MIN_GROUP_SIZE:
        return []
    centers = np.array([get_track_center(t) for t in falling_candidates])
    final_tracks = []
    for i in range(len(falling_candidates)):
        current_center = centers[i]
        distances = np.linalg.norm(centers - current_center, axis=1)
        neighbor_count = np.sum((distances < GROUP_RADIUS) & (distances > 0))
        if neighbor_count >= MIN_GROUP_SIZE:
            final_tracks.append(falling_candidates[i])
    return final_tracks

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
    width_2_9 = RESIZE_WIDTH * 2 // 9
    width_7_9 = RESIZE_WIDTH * 7 // 9
    
    # 準備
    resized_first = cv2.resize(first_frame, (RESIZE_WIDTH, resize_h), interpolation=cv2.INTER_AREA)
    old_gray = cv2.cvtColor(resized_first, cv2.COLOR_BGR2GRAY)
    
    active_tracks = []
    waterfall_memory = {}
    
    fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
    
    # 計測用変数
    frame_count = 0
    start_time = time.time()
    prev_time = start_time
    process_times = [] # 処理時間の履歴
    
    all_process_times = []
    
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

            # マスク準備
            mask = np.zeros_like(latest_resized) if SHOW_WINDOW else None

            # オプティカルフロー
            new_tracks = []
            if active_tracks:
                p0 = np.float32([tr[-1] for tr in active_tracks]).reshape(-1, 1, 2)
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                
                for i, (track, (new_point, status)) in enumerate(zip(active_tracks, zip(p1, st))):
                    if status[0] == 0: 
                        continue
                    new_point_tuple = tuple(new_point.ravel())
                    if width_2_9 < new_point_tuple[0] < width_7_9:
                        continue 
                    track.append(new_point_tuple)
                    if len(track) > TRACK_MAX_LEN: 
                        track.pop(0)
                    new_tracks.append(track)
            active_tracks = new_tracks

            # 特徴点追加
            if len(active_tracks) < MIN_TRACKS or frame_count % RE_DETECT_INTERVAL == 0:
                detection_mask = np.zeros(frame_gray.shape, dtype=np.uint8)
                detection_mask[:, 0:width_2_9] = 255
                detection_mask[:, width_7_9:RESIZE_WIDTH] = 255
                for track in active_tracks:
                    if track: 
                        cv2.circle(detection_mask, (int(track[-1][0]), int(track[-1][1])), NEW_POINT_MIN_DIST, 0, -1)
                
                keypoints = fast.detect(frame_gray, mask=detection_mask)
                if keypoints:
                    keypoints = sorted(keypoints, key=lambda x: x.response, reverse=True)
                    for kp in keypoints[:feature_params['maxCorners']]:
                        active_tracks.append([(kp.pt[0], kp.pt[1])])

            # 落下判定
            falling_candidates = []
            for track in active_tracks:
                if len(track) < 2: 
                    continue
                if judge_falling(track): 
                    falling_candidates.append(track)
            passed_tracks = find_dense_falling_tracks(falling_candidates)
            
            # --- メモリ更新 ---
            expired_cells = []
            for cell in waterfall_memory:
                waterfall_memory[cell] -= 1
                if waterfall_memory[cell] <= 0: 
                    expired_cells.append(cell)
            for cell in expired_cells: 
                del waterfall_memory[cell]
            for track in passed_tracks:
                if not track: continue
                end_point = track[-1]
                key = (int(end_point[0]), int(end_point[1]))
                waterfall_memory[key] = DETECTION_TTL
            
            # 次フレーム準備
            old_gray = frame_gray.copy()
            frame_count += 1
            
            # --- 計測終了 (処理単体) ---
            proc_end = time.perf_counter()
            proc_time_ms = (proc_end - proc_start) * 1000
            #process_times.append(proc_time_ms)
            #if len(process_times) > 100: process_times.pop(0)
            all_process_times.append(proc_time_ms) # 全履歴に追加（捨てない）
            
            
            # 画面表示
            if SHOW_WINDOW and not SKIP_RENDERING:
                if passed_tracks:
                    cv2.polylines(mask, [np.int32(t) for t in passed_tracks], False, (255, 100, 0), 2)
                img = cv2.add(latest_resized, mask)
                #avg_proc = sum(process_times)/len(process_times)
                current_avg = sum(all_process_times[-100:]) / len(all_process_times[-100:])
                #cv2.putText(img, f"Proc: {proc_time_ms:.1f}ms", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(img, f"Proc: {proc_time_ms:.1f}ms (Avg: {current_avg:.1f})", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow('Camera Benchmark', img)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            # FPS計測とコンソール出力 (1秒ごと)
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                #avg_proc = sum(process_times)/len(process_times) if process_times else 0
                # リストの末尾100個だけを取り出して平均を計算
                recent_avg = sum(all_process_times[-100:]) / len(all_process_times[-100:]) if all_process_times else 0
                print(f"FPS: {fps:.2f} | Avg Proc Time: {recent_avg:.2f}ms | Tracks: {len(active_tracks)}")
                frame_count = 0
                prev_time = curr_time

    except KeyboardInterrupt:
        print("Stopped by user.")
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