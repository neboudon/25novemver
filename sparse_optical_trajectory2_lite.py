import cv2
import numpy as np
import os
import sys

# --- 1. パラメータ設定 (変更なし) ---
# ... (VIDEO_FOLDER_PATH, PLAYBACK_SPEED_MS, RESIZE_WIDTH) ...
VIDEO_FOLDER_PATH = "/Users/shigemitsuhiroki/vscode/sewage_movie/9_3_movie"
PLAYBACK_SPEED_MS = 30
RESIZE_WIDTH = 480

# --- 2. 軌跡追跡のパラメータ (変更なし) ---
# ... (TRACK_MAX_LEN, RE_DETECT_INTERVAL, MIN_TRACKS, NEW_POINT_MIN_DIST) ...
TRACK_MAX_LEN = 50
RE_DETECT_INTERVAL = 10
MIN_TRACKS = 50
NEW_POINT_MIN_DIST = 15

# --- 3. 滝の検出パラメータ (変更なし) ---
# ... (TRAJECTORY_MIN_DY, TRAJECTORY_DRIFT_RATIO, TRAJECTORY_MIN_POINTS) ...
# 軌跡の最小垂直移動量 (短くても検出するため値を小さくする)
TRAJECTORY_MIN_DY = 20.0 
TRAJECTORY_DRIFT_RATIO = 0.3
# 軌跡を分析するために必要な最小点数 (短い軌跡を許容するため値を小さくする)
TRAJECTORY_MIN_POINTS = 5

# --- 4. クラスタリングと「メモリ」のパラメータ (変更・追加) ---
CLUSTER_MIN_TRACKS = 3
CLUSTER_GRID_CELL_SIZE = 40

# ★★★ 追加: 検出メモリ（ちらつき防止） ★★★
# 一度検出した滝を何フレーム記憶するか (Time-to-Live)
DETECTION_TTL = 15  # 15フレーム (30fpsなら0.5秒) 記憶する


# --- 5. OpenCVパラメータ (変更なし) ---
# ... (feature_params, lk_params) ...
feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=10, blockSize=7)
lk_params = dict(winSize=(13, 13), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


# --- analyze_trajectory 関数 (変更なし) ---
def analyze_trajectory(track):
    """
    1本の軌跡(track)を分析し、滝かどうかを判定する
    """
    if len(track) < TRAJECTORY_MIN_POINTS:
        return False
    start_point = track[0]
    end_point = track[-1]
    start_x, start_y = start_point
    end_x, end_y = end_point
    dx = end_x - start_x
    dy = end_y - start_y
    if dy < TRAJECTORY_MIN_DY:
        return False
    if dy == 0:
        return False
    drift_ratio = abs(dx) / dy
    if drift_ratio > TRAJECTORY_DRIFT_RATIO:
        return False
    return True


# --- ★★★ 修正: find_clusters 関数 (戻り値を変更) ★★★ ---
def find_clusters(candidate_tracks, grid_size, width, height, min_tracks):
    """
    滝候補の軌跡リストから、密集領域（ホットなグリッドセル）を見つける
    
    Returns:
        set: (cell_y, cell_x) のタプルを含むセット
             (例: {(10, 5), (10, 6)})
    """
    if not candidate_tracks:
        return set() # 空のセットを返す

    # 密集をカウントするためのグリッドを作成
    grid_w = int(np.ceil(width / grid_size))
    grid_h = int(np.ceil(height / grid_size))
    grid_count = np.zeros((grid_h, grid_w), dtype=int)
    
    # 軌跡の中間点をグリッドにマッピングしてカウント
    for track in candidate_tracks:
        mid_point = track[len(track) // 2]
        mid_x, mid_y = int(mid_point[0]), int(mid_point[1])
        
        grid_x = mid_x // grid_size
        grid_y = mid_y // grid_size
        
        if 0 <= grid_y < grid_h and 0 <= grid_x < grid_w:
            grid_count[grid_y, grid_x] += 1
            
    # しきい値(min_tracks)を超えたグリッド（「熱い」セル）を探す
    hot_cells_np = np.argwhere(grid_count >= min_tracks)
    
    # (y, x) タプルのセットとして返す
    hot_cells_set = set(tuple(cell) for cell in hot_cells_np)
        
    return hot_cells_set


def main():
    # --- 1. 動画ファイルの選択 (省略... 元のコードと同じ) ---
    try:
        video_files = [f for f in os.listdir(VIDEO_FOLDER_PATH) if f.lower().endswith(('.mp4', '.avi', '.mov', '.mkv'))]
        if not video_files: print(f"エラー: フォルダ '{VIDEO_FOLDER_PATH}' に動画ファイルが見つかりません。"); return
        print("--- 処理する動画を選択してください ---")
        for i, filename in enumerate(video_files): print(f"  {i}: {filename}")
        choice = int(input("番号を入力してください: "))
        selected_video = video_files[choice]
        video_path = os.path.join(VIDEO_FOLDER_PATH, selected_video)
        print(f"'{selected_video}' を処理します。")
    except (FileNotFoundError, IndexError, ValueError) as e: print(f"エラー: 動画の選択に失敗しました。({e})"); return

    # --- 2. 動画の読み込みと情報表示 (省略... 元のコードと同じ) ---
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened(): print(f"エラー: 動画ファイル '{video_path}' を開けません。"); return
    frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT); fps = cap.get(cv2.CAP_PROP_FPS); video_duration_sec = 0
    if fps > 0: video_duration_sec = frame_count / fps; minutes = int(video_duration_sec // 60); seconds = int(video_duration_sec % 60); print(f"動画の長さ: {minutes}分{seconds}秒 ({fps:.2f} FPS)")
    else: print("動画の長さを取得できませんでした。")
        
    # --- 3. ユーザーから再生開始時間を取得 (省略... 元のコードと同じ) ---
    start_frame = 0
    if video_duration_sec > 0: 
        while True:
            try:
                start_min_str = input("再生を開始する時間（分）を入力してください (例: 1): "); start_sec_str = input("再生を開始する時間（秒）を入力してください (例: 30): "); start_min = int(start_min_str); start_sec = int(start_sec_str); total_input_seconds = start_min * 60 + start_sec
                if total_input_seconds >= video_duration_sec: print(f"エラー: 入力された時間は動画の長さを超えています。再度入力してください。"); continue
                start_frame = int(total_input_seconds * fps); print(f"{start_min}分{start_sec}秒（{start_frame}フレーム目）から再生を開始します。"); break
            except ValueError: print("エラー: 半角数字で入力してください。")
            except Exception as e: print(f"予期せずエラーが発生しました: {e}"); return
    else: print("動画長の取得に失敗したため、最初から再生します。")

    # --- 4. メインループで動画を再生・処理 ---
    print("\nビデオの処理を開始します。")
    print("  - qキー: 終了")
    print("  - スペースキー: 一時停止 / 再生")
    
    paused = False
    
    try:
        # --- 5. 開始フレームの設定と最初のフレーム処理 ---
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        ret, first_frame = cap.read()
        if not ret: print("指定された開始フレームを読み込めませんでした。"); return
        
        orig_height, orig_width = first_frame.shape[:2]
        aspect_ratio = orig_height / orig_width
        resize_height = int(RESIZE_WIDTH * aspect_ratio)
        resized_first_frame = cv2.resize(first_frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
        old_gray = cv2.cvtColor(resized_first_frame, cv2.COLOR_BGR2GRAY)
        
        # --- 軌跡追跡のロジック ---
        active_tracks = [] 
        frame_idx = 0
        
        # --- ★★★ 追加: 滝検出の「メモリ」 ★★★ ---
        # キー: (grid_y, grid_x), 値: TTL (Time-to-Live)
        waterfall_memory = {}
        
        # --- 6. メインループ ---
        while True:
            if not paused:
                ret, frame = cap.read()
                if not ret: print("\nビデオが終了しました。"); break
                
                resized_frame = cv2.resize(frame, (RESIZE_WIDTH, resize_height), interpolation=cv2.INTER_AREA)
                frame_gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
                mask = np.zeros_like(resized_frame)
                new_tracks = []
                
                # --- 6a. 既存の軌跡を追跡 (変更なし) ---
                if active_tracks:
                    #なにやってるのか？
                    p0 = np.float32([tr[-1] for tr in active_tracks]).reshape(-1, 1, 2)
                    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                    for i, (track, (new_point, status)) in enumerate(zip(active_tracks, zip(p1, st))):
                        #status(追跡成功)が1のときのみ更新
                        if status[0] == 0: continue
                        new_point_tuple = tuple(new_point.ravel())
                        track.append(new_point_tuple)
                        if len(track) > TRACK_MAX_LEN: track.pop(0)
                        new_tracks.append(track)
                active_tracks = new_tracks

                # --- 6b. 新しい特徴点を検出・追加 (変更なし) ---
                if len(active_tracks) < MIN_TRACKS or frame_idx % RE_DETECT_INTERVAL == 0:
                    detection_mask = np.full(frame_gray.shape, 255, dtype=np.uint8)
                    for track in active_tracks: cv2.circle(detection_mask, (int(track[-1][0]), int(track[-1][1])), NEW_POINT_MIN_DIST, 0, -1)
                    new_points = cv2.goodFeaturesToTrack(old_gray, mask=detection_mask, **feature_params)
                    if new_points is not None:
                        for p in new_points: active_tracks.append([tuple(p.ravel())])
                
                # --- 6c. 全軌跡を分析・描画 (変更なし) ---
                candidate_tracks = [] 
                for track in active_tracks:
                    if len(track) < 2: continue
                    if analyze_trajectory(track):
                        #候補の軌跡が条件を満たしていれば、青い線で描画
                        candidate_tracks.append(track)
                        cv2.polylines(mask, [np.int32(track)], isClosed=False, color=(255, 100, 0), thickness=2)
                    else:
                        cv2.polylines(mask, [np.int32(track)], isClosed=False, color=(0, 255, 0), thickness=1)

                # --- ★★★ 修正: 6d. 検出メモリ（TTL）の更新 ★★★ ---
                
                # (1) このフレームで「熱い」グリッドセルを検出
                current_hot_cells = find_clusters(
                    candidate_tracks, 
                    CLUSTER_GRID_CELL_SIZE, 
                    RESIZE_WIDTH, 
                    resize_height, 
                    CLUSTER_MIN_TRACKS
                )
                
                # (2) メモリ内の既存セルのTTLを 1 減らす
                expired_cells = []
                for cell in waterfall_memory:
                    waterfall_memory[cell] -= 1
                    if waterfall_memory[cell] <= 0:
                        expired_cells.append(cell)
                
                # (3) TTLが切れた（0になった）セルをメモリから削除
                for cell in expired_cells:
                    del waterfall_memory[cell]
                    
                # (4) 現在検出された「熱い」セルのTTLを最大値(DETECTION_TTL)でリセット/追加
                for cell in current_hot_cells:
                    waterfall_memory[cell] = DETECTION_TTL
                      
                # --- ★★★ 修正: 7. 結果の表示 & 標準出力 ★★★ ---
                
                display_frame = resized_frame.copy()
                img = cv2.add(display_frame, mask)
                
                # メモリに滝が記憶されている場合のみ描画・出力
                if waterfall_memory:
                    # コンソールのクリア（見やすくするため。不要なら削除）
                    print("\033[2J\033[H", end="") 
                    print("--- STABLE WATERFALL DETECTIONS (座標出力) ---")
                    
                    for (cell_y, cell_x), ttl in waterfall_memory.items():
                        # グリッド座標をピクセル座標（円の中心）に変換
                        radius = CLUSTER_GRID_CELL_SIZE // 2
                        center_x = int((cell_x + 0.5) * CLUSTER_GRID_CELL_SIZE)
                        center_y = int((cell_y + 0.5) * CLUSTER_GRID_CELL_SIZE)
                        
                        # ★★★ ご要望の「標準出力」 ★★★
                        print(f"  REGION (x={center_x}, y={center_y}), [TTL remaining: {ttl}]")

                        # 確定した滝の密集領域を赤丸で表示
                        cv2.circle(img, (center_x, center_y), radius, (0, 0, 255), 3) # 赤い円
                        cv2.putText(img, "WATERFALL", (center_x - radius, center_y - radius - 5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.imshow('Original Video', resized_frame)
                cv2.imshow('Waterfall Trajectory Detection', img)
                
                # --- 8. 次のフレームの準備 (変更なし) ---
                old_gray = frame_gray.copy()
                frame_idx += 1
                
            # --- 9. キー入力処理 (変更なし) ---
            key = cv2.waitKey(PLAYBACK_SPEED_MS) & 0xFF
            if key == ord('q'): print("\n処理を中断しました。"); break
            elif key == ord(' '): paused = not paused
                
    finally:
        # --- 10. 終了処理 (変更なし) ---
        cap.release()
        cv2.destroyAllWindows()
        print("ビデオを解放し、ウィンドウを閉じました。")

if __name__ == '__main__':
    main()