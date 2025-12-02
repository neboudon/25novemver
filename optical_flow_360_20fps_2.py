import cv2
import numpy as np
import os
import sys
import time

# --- 1. パラメータ設定 ---
VIDEO_FOLDER_PATH = "/Users/shigemitsuhiroki/vscode/sewage_movie/9_3_movie"
PLAYBACK_SPEED_MS = 40
RESIZE_WIDTH = 360

# --- 2. 軌跡追跡のパラメータ ---
TRACK_MAX_LEN = 50
RE_DETECT_INTERVAL = 10
MIN_TRACKS = 40
NEW_POINT_MIN_DIST = 15

# --- 3. 滝の検出パラメータ ---
TRAJECTORY_MIN_DY = 20.0 
TRAJECTORY_DRIFT_RATIO = 0.5
TRAJECTORY_MIN_POINTS = 5

# --- 4. クラスタリングと「メモリ」のパラメータ ---
CLUSTER_MIN_TRACKS = 3
CLUSTER_GRID_CELL_SIZE = 40
DETECTION_TTL = 15

# --- 5. OpenCVパラメータ ---
feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=10, blockSize=7)
lk_params = dict(winSize=(10, 10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

#落下判定のパラメータ
FALL_MIN_DY = 10.0
MOVE_X_MAX = 10.0

#周囲判定のパラメータ
GROUP_RADIUS = 40.0
MIN_GROUP_SIZE = 2

# --- 関数群 (変更なし) ---
def analyze_trajectory(track):
    start_point = track[0]; end_point = track[-1]
    start_x, start_y = start_point; end_x, end_y = end_point
    dx = end_x - start_x; dy = end_y - start_y
    if dy < TRAJECTORY_MIN_DY: return False
    if dy == 0: return False
    drift_ratio = abs(dx) / dy
    if drift_ratio > TRAJECTORY_DRIFT_RATIO: return False
    return True

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
    
def find_clusters(candidate_tracks, grid_size, width, height, min_tracks):
    if not candidate_tracks: return set()
    grid_w = int(np.ceil(width / grid_size))
    grid_h = int(np.ceil(height / grid_size))
    grid_count = np.zeros((grid_h, grid_w), dtype=int)
    for track in candidate_tracks:
        mid_point = track[len(track) // 2]
        mid_x, mid_y = int(mid_point[0]), int(mid_point[1])
        grid_x = mid_x // grid_size; grid_y = mid_y // grid_size
        if 0 <= grid_y < grid_h and 0 <= grid_x < grid_w:
            grid_count[grid_y, grid_x] += 1
    hot_cells_np = np.argwhere(grid_count >= min_tracks)
    hot_cells_set = set(tuple(cell) for cell in hot_cells_np)
    return hot_cells_set


def main():
    # --- 1. 動画ファイルの選択 ---
    try:
        video_files = [f for f in os.listdir(VIDEO_FOLDER_PATH) if f.lower().endswith(('.mp4', '.avi', '.mov', '.mkv'))]
        if not video_files:
            print(f"エラー: フォルダ '{VIDEO_FOLDER_PATH}' に動画ファイルが見つかりません。")
            return
        print("--- 処理する動画を選択してください ---")
        for i, filename in enumerate(video_files): print(f"  {i}: {filename}")
        choice = int(input("番号を入力してください: "))
        selected_video = video_files[choice]
        video_path = os.path.join(VIDEO_FOLDER_PATH, selected_video)
        print(f"'{selected_video}' を処理します。")
    except (FileNotFoundError, IndexError, ValueError) as e: 
        print(f"エラー: 動画の選択に失敗しました。({e})")
        return

    # --- 2. 動画の読み込みと情報表示 ---
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened(): 
        print(f"エラー: 動画ファイル '{video_path}' を開けません。")
        return
    frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    
    source_fps = cap.get(cv2.CAP_PROP_FPS)
    print("FPS", source_fps)
    TARGET_FPS = 50.0
    PROCESS_EVERY_N_FRAMES = 2
    
    if source_fps <= 0: source_fps = 30.0
    frame_step = source_fps / TARGET_FPS
    if frame_step < 1.0: frame_step = 1.0
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    video_duration_sec = 0
    if fps > 0: 
        video_duration_sec = frame_count / fps
        minutes = int(video_duration_sec // 60)
        seconds = int(video_duration_sec % 60)
        print(f"動画の長さ: {minutes}分{seconds}秒 ({fps:.2f} FPS)")
    else: 
        print("動画の長さを取得できませんでした。")
        
    # --- 3. 開始時間取得 ---
    start_frame = 0
    if video_duration_sec > 0: 
        while True:
            try:
                start_min_str = input("再生を開始する時間（分）を入力してください (例: 1): ")
                start_sec_str = input("再生を開始する時間（秒）を入力してください (例: 30): ")
                start_min = int(start_min_str); start_sec = int(start_sec_str)
                total_input_seconds = start_min * 60 + start_sec
                if total_input_seconds >= video_duration_sec: 
                    print(f"エラー: 入力された時間は動画の長さを超えています。")
                    continue
                start_frame = int(total_input_seconds * fps)
                print(f"{start_min}分{start_sec}秒（{start_frame}フレーム目）から再生を開始します。")
                break
            except ValueError:
                print("エラー: 半角数字で入力してください。")
            except Exception as e: 
                print(f"予期せずエラー: {e}")
                return
    else: 
        print("最初から再生します。")

    # --- 4. メインループ準備 ---
    print("\nビデオの処理を開始します。")
    print("  - qキー: 終了")
    print("  - スペースキー: 一時停止 / 再生")
    
    paused = False
    
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        ret, first_frame = cap.read()
        if not ret: 
            print("指定された開始フレームを読み込めませんでした。"); 
            return
        
        orig_height, orig_width = first_frame.shape[:2]
        aspect_ratio = orig_height / orig_width
        resize_height = int(RESIZE_WIDTH * aspect_ratio)
        
        width_2_9 = RESIZE_WIDTH * 2 // 9
        width_7_9 = RESIZE_WIDTH * 7 // 9
        
        resized_first_frame = cv2.resize(first_frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
        old_gray = cv2.cvtColor(resized_first_frame, cv2.COLOR_BGR2GRAY)
        
        active_tracks = [] 
        frame_idx = 0
        waterfall_memory = {}
        detection_log = []
        
        last_flow_time = time.time()
        
        mask = np.zeros_like(resized_first_frame)
        latest_resized_frame = resized_first_frame.copy()
        
        frame_accumulator = 0.0
        
        # --- 6. メインループ ---
        read_frame_idx = 0
        while True:
            if not paused:
                # フレーム進行計算
                frame_accumulator += frame_step
                frames_to_advance = int(frame_accumulator)
                if frames_to_advance == 0:
                    frames_to_advance = 1
                frame_accumulator -= frames_to_advance
                
                # 読み飛ばし
                for _ in range(frames_to_advance - 1):
                    cap.grab()
                
                ret, frame = cap.read()
                if not ret: 
                    print("\nビデオが終了しました。"); 
                    break
                
                read_frame_idx += 1
                
                # 2回に1回処理 (それ以外はスキップ)
                if read_frame_idx % PROCESS_EVERY_N_FRAMES != 0:
                    continue
                
                # ここから画像処理（インデントを正しく揃える）
                latest_resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
                
                # 時間更新 (if文を外して毎回更新)
                last_flow_time = time.time()
                
                frame_gray = cv2.cvtColor(latest_resized_frame, cv2.COLOR_BGR2GRAY)
                mask = np.zeros_like(latest_resized_frame)
                new_tracks = []
                
                # --- オプティカルフロー計算 ---
                if active_tracks:
                    p0 = np.float32([tr[-1] for tr in active_tracks]).reshape(-1, 1, 2)
                    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                    for i, (track, (new_point, status)) in enumerate(zip(active_tracks, zip(p1, st))):
                        if status[0] == 0:
                            continue
                        
                        new_point_tuple = tuple(new_point.ravel())
                        new_x = new_point_tuple[0]
                        
                        # 領域外（中央部分）なら追跡中止
                        if width_2_9 < new_x < width_7_9:
                            continue 
                        
                        track.append(new_point_tuple)
                        if len(track) > TRACK_MAX_LEN: 
                            track.pop(0)
                        new_tracks.append(track)
                active_tracks = new_tracks

                # --- 新規特徴点検出 ---
                if len(active_tracks) < MIN_TRACKS or frame_idx % RE_DETECT_INTERVAL == 0:
                    detection_mask = np.zeros(frame_gray.shape, dtype=np.uint8)
                    detection_mask[:, 0:width_2_9] = 255
                    detection_mask[:, width_7_9:RESIZE_WIDTH] = 255
                    
                    for track in active_tracks: 
                        if track:
                            cv2.circle(detection_mask, (int(track[-1][0]), int(track[-1][1])), NEW_POINT_MIN_DIST, 0, -1)
                    
                    new_points = cv2.goodFeaturesToTrack(frame_gray, mask=detection_mask, **feature_params)
                    if new_points is not None:
                        for p in new_points: 
                            active_tracks.append([tuple(p.ravel())])
            
                # --- 分析と描画 (ここからインデントを修正しました) ---
                falling_candidates = []
                other_candidates = [] 
                for track in active_tracks:
                    if len(track) < 2: continue
                    if judge_falling(track):
                        falling_candidates.append(track)
                    else:
                        other_candidates.append(track)
                
                passed_tracks = find_dense_falling_tracks(falling_candidates)
                isolated_falling_tracks = [t for t in falling_candidates if t not in passed_tracks]
                
                if passed_tracks:
                        cv2.polylines(mask, [np.int32(t) for t in (passed_tracks)], isClosed=False, color=(255, 100, 0), thickness=2)
                if isolated_falling_tracks:
                        cv2.polylines(mask, [np.int32(t) for t in (isolated_falling_tracks)], isClosed=False, color=(0, 0, 255), thickness=1)
                if other_candidates:
                        cv2.polylines(mask, [np.int32(t) for t in (other_candidates)], isClosed=False, color=(0, 255, 0), thickness=1)

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
                    
                # --- 結果表示 ---
                old_gray = frame_gray.copy()
                frame_idx += 1
                img = cv2.add(latest_resized_frame, mask)
                
                overlay = img.copy()
                cv2.rectangle(overlay, (width_2_9, 0), (width_7_9, resize_height), (0,0,0), -1)
                cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)
                
                if waterfall_memory:
                    current_pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
                    current_seconds = current_pos_frame / fps
                    curr_min = int(current_seconds // 60)
                    curr_sec = int(current_seconds % 60)
                    time_str = f"{curr_min:02d}:{curr_sec:02d}"
                    if not detection_log or detection_log[-1] != time_str:
                        detection_log.append(time_str)
                    print("\033[2J\033[H", end="") 
                    print(f"--- STABLE WATERFALL DETECTIONS (Time: {time_str}) ---")
                    for (center_x, center_y), ttl in waterfall_memory.items():
                        print(f"  REGION (x={center_x}, y={center_y}), [TTL remaining: {ttl}]")

                cv2.imshow('Waterfall Trajectory Detection (Region Limited)', img) 
            
            # --- キー入力 (if not pausedの外) ---
            key = cv2.waitKey(PLAYBACK_SPEED_MS) & 0xFF
            if key == ord('q'): print("\n処理を中断しました。"); break
            elif key == ord(' '): paused = not paused
                
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("ビデオを解放し、ウィンドウを閉じました。")
        if detection_log:
            print("\n=== 水漏れ検出履歴 ===")
            for t_str in detection_log:
                print(f"Detected waterfall at {t_str}")
        else:
            print("\n=== 水漏れは検出されませんでした ===")

if __name__ == '__main__':
    main()