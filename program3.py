import cv2
import pyrealsense2 as rs
import threading
import time
import numpy as np
import serial
import math

#重心と消失点検出用のパラメータ
RESIZE_WIDTH = 240

#シリアル通信用パラメータ
SERIAL_PORT = '/dev/ttyS0' 
SERIAL_BAUDRATE = 921600  # Pico側もこれに合わせてください

#func1からの関数の呼び出し
from func3 import send_motor_command, process_cog, process_mis,process_obstacle_detection

#カメラ画像取得用のクラス
class CameraStream:
    def __init__(self):
        self.lock = threading.Lock()
        self.capture_time = 0
        color_w, color_h = 640, 360
        depth_w, depth_h = 640, 360
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, 60)
        profile = self.pipeline.start(self.config)
        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.latest_frame = None
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.capture_time = 0
        
    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        
    def update(self):
        while True:
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            now = time.time()
            
            with self.lock:
                self.latest_frame = frames
                self.latest_color_frame = color_image
                self.latest_depth_frame = depth_image
                self.capture_time = now
    
    def get_latest(self):
        with self.lock:
            return self.latest_frame, self.latest_color_frame, self.latest_depth_frame, self.capture_time


#measure_dist2_1_1.pyのために作成した関数
# --- マスク作成用ヘルパー ---
def get_top_x(bottom_x, angle_deg, dy, direction="left"):
    dx = int(dy / math.tan(math.radians(angle_deg)))
    return bottom_x + dx if direction == "left" else bottom_x - dx

def create_mask(pts, h, w):
    m = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
    return m
        
        
#メイン関数
def main():
    camera = CameraStream()
    camera.start()
    
    #処理レート(秒)
    INTERVAL_COG = 0.5
    INTERVAL_OBSTACLE = 1.0 / 15.0
    
    #ループ全体の処理時間
    LOOP_PERIOD = 0.025 # 40Hzの基本周期
    
    # キャリブレーション設定
    CALIB_FRAMES_LIMIT = 10
    calib_counter = 0
    accum_depth = None
    ref_depth = None
    
    # 障害物検出用マスクの初期化 (measure_dist2_1_1 のパラメータ)
    h, w = 360, 640
    y_top = int(h * (2/3))
    dy = h - y_top
    
    p_lo_b, p_lo_t = (50, h), (get_top_x(50, 30, dy, "left"), y_top)
    p_li_b, p_li_t = (220, h), (get_top_x(220, 55, dy, "left"), y_top)
    p_ri_b, p_ri_t = (w - 170, h), (get_top_x(w - 170, 50, dy, "right"), y_top)
    p_ro_b, p_ro_t = (w - 0, h), (get_top_x(w - 0, 30, dy, "right"), y_top)

    lane_masks = {
        "LEFT":   create_mask([p_lo_b, p_lo_t, p_li_t, p_li_b], h, w),
        "CENTER": create_mask([p_li_b, p_li_t, p_ri_t, p_ri_b], h, w),
        "RIGHT":  create_mask([p_ri_b, p_ri_t, p_ro_t, p_ro_b], h, w)
    }
    total_lane_mask = cv2.bitwise_or(cv2.bitwise_or(lane_masks["LEFT"], lane_masks["CENTER"]), lane_masks["RIGHT"])
    
    #消失点検出のためのパラメータ
    CLIP_LIMIT = 15.0
    TILE_GRID_SIZE = (4, 4)
    
    #時間管理変数
    last_time_cog = 0
    last_time_obstacle = 0
    
    while True:
        _, first_color_frame, first_depth_frame, _ = camera.get_latest()
        if first_depth_frame is None or first_color_frame is None:
            print("カメラの起動中...")
            time.sleep(0.1)
            continue
        break
    
    #重心と消失点検出用のリサイズの高さを計算
    orig_h, orig_w = first_color_frame.shape[:2]
    aspect_ratio = orig_h / orig_w
    resize_h = int(RESIZE_WIDTH * aspect_ratio)
    
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: ポート {SERIAL_PORT} (Baud: {SERIAL_BAUDRATE}) を開きました。")
        time.sleep(2)  # シリアルポートの初期化待ち
    
    except serial.SerialException as e:
        print("シリアルポートのオープンに失敗しました。")
        return
    
    print("制御を開始します")
    
    #変数の初期化
    clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_GRID_SIZE)
    align = rs.align(rs.stream.color)
    result_img = np.zeros((resize_h, RESIZE_WIDTH, 3), dtype=np.uint8)
    
    while True:
        loop_start_time = time.time()
        
        #画像の取得
        frame, color_img, depth_img , img_time = camera.get_latest()
        if color_img is None: continue
        
        # --- キャリブレーションフェーズ (最初の10フレーム) ---
        if calib_counter < CALIB_FRAMES_LIMIT:
            # ここでだけアライメントを実行
            aligned_frames = align.process(frame)
            aligned_color_frame = aligned_frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame()
            
            if not aligned_depth_frame or not aligned_color_frame:continue 
            
            #depth_cm = aligned_depth_frame.astype(float) * 0.1
            depth_cm = np.asanyarray(aligned_depth_frame.get_data()).astype(float) * 0.1
            color_img_calib = np.asanyarray(aligned_color_frame.get_data())
            if accum_depth is None: accum_depth = np.zeros((h, w), dtype=float)
            accum_depth = np.nansum([accum_depth, depth_cm], axis=0)
            calib_counter += 1
            
            #send_motor_command(ser, "S\n") # キャリブ中も停止命令
            cv2.putText(color_img_calib, f"CALIBRATING... {calib_counter}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.imshow("result_img", color_img_calib)
            if calib_counter == CALIB_FRAMES_LIMIT:
                ref_depth = accum_depth / CALIB_FRAMES_LIMIT
            cv2.waitKey(1)
            continue
        
                #フラグの初期化
        send_command_flag = False #コマンドを送るか？
        emergency_flag = False #カメラのフレームが遅れているか？
        
        """
        if(loop_start_time - img_time) > 0.1:
            print("警告:映像遅延．停止します．")
            emergency_flag = True
        """ 
        
        # 1. 障害物検出 (15Hz)
        if (loop_start_time - last_time_obstacle) >= INTERVAL_OBSTACLE:
            aligned_frames = align.process(frame)
            aligned_color_frame = aligned_frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame()
            
            if aligned_color_frame and aligned_depth_frame:
                color_img_aligned = np.asanyarray(aligned_color_frame.get_data())
                depth_img_aligned = np.asanyarray(aligned_depth_frame.get_data())
                depth_cm = depth_img_aligned.astype(float) * 0.1
                
                # マスク外と0(欠損値)をNaNにする
                depth_cm[total_lane_mask == 0] = np.nan
                depth_cm[depth_cm == 0] = np.nan
            
                obs_img, obs_info = process_obstacle_detection(color_img_aligned, depth_cm, ref_depth, lane_masks)
            
                # A. 計測エリアの半透明グレー塗り
                #overlay = color_img_aligned
                #cv2.fillPoly(overlay, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], (100, 100, 100))
                #obs_img = cv2.addWeighted(overlay, 0.3, obs_img, 0.7, 0)
                
                # B. 外枠の描画（水色/黄色）
                cv2.polylines(obs_img, [np.array([p_lo_b, p_lo_t, p_ro_t, p_ro_b], np.int32)], True, (255, 255, 0), 2)
                
                # C. 内側の境界線の描画（黄色）
                cv2.line(obs_img, p_li_b, p_li_t, (0, 255, 255), 2) # 左内境界
                cv2.line(obs_img, p_ri_b, p_ri_t, (0, 255, 255), 2) # 右内境界
                
                # ここで obs_info を使った回避ロジックを将来的に書けます
                # 例: if any(d['lanes'] == ['CENTER'] for d in obs_info): ...
                
                cv2.imshow("Obstacle Detection", obs_img)
            last_time_obstacle = loop_start_time
        
        elif((loop_start_time - last_time_cog) >= INTERVAL_COG):
            
            #重心検出の実行
            #final_cmd = process_cog(color_img,resize_h)
            #final_cmd,result_img = process_cog(color_img,resize_h)
            
            #消失点検出の実行
            #final_cmd = process_mis(color_img, resize_h, clahe)
            final_cmd,result_img = process_mis(color_img, resize_h, clahe)
            
            #フラグの更新
            last_time_cog = loop_start_time #最終処理時間の更新
            send_command_flag = True #コマンド送信のフラグを立てる
            
        else:
            pass
        
        cv2.imshow("result_img",result_img)
        key = cv2.waitKey(1)
        if key == 27: # ESCキーで終了
            break
        
        if emergency_flag == True:
            final_cmd = "S\n" #ここは修正する
            #コマンド送信
            send_motor_command(ser,final_cmd)
            flow_result = None
            
        elif send_command_flag == True:
            #コマンド送信
            send_motor_command(ser,final_cmd)
            flow_result = final_cmd
        
        elapsed = time.time() - loop_start_time
        sleep_time = LOOP_PERIOD - elapsed
        
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    main()