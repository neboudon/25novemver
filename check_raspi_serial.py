import time
import serial
import threading

# 高速通信設定
SERIAL_PORT = '/dev/ttyS0' 
SERIAL_BAUDRATE = 921600  # Pico側もこれに合わせてください

def main():
    ser = None
    frame_count = 0
    process_times = []
    
    try:
        # timeout=1.0 は「応答が来ない場合に1秒で諦める」設定です
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: ポート {SERIAL_PORT} (Baud: {SERIAL_BAUDRATE}) を開きました。")
        print("[メイン]: 送信＆応答確認を開始します...(Ctrl+Cで停止)")
        
    except serial.SerialException as e:
        print(f"[エラー]: ポートを開けません。{e}")
        return

    prev_time = time.time()
    
    try:
        while True:
            # 送信するコマンド
            direction = "Lw"
            error = 15.0
            command = f"{direction} {error:.2f}\n"
            
            # ==========================================
            # 1. 送信時間の計測 (ここだけ測りたい)
            # ==========================================
            proc_start = time.perf_counter()
            
            # バッファに書き込み & 物理送信完了まで待機
            ser.write(command.encode('utf-8'))
            ser.flush()
            
            # ★ここで計測終了 (受信時間は含めない)
            proc_end = time.perf_counter()
            
            # ==========================================
            # 2. 応答の受け取り (計測対象外)
            # ==========================================
            try:
                # Picoからの "OK: ..." を待つ
                response = ser.readline().decode('utf-8').strip()
                
                # もしデバッグで応答の中身を見たい場合はコメントを外す
                # if not response:
                #    print("[警告] 応答なし(Timeout)")
                
            except serial.SerialException:
                print("[エラー] 読み込み失敗")
                break

            # ==========================================
            # 3. 統計処理
            # ==========================================
            proc_time_ms = (proc_end - proc_start) * 1000
            process_times.append(proc_time_ms)
            frame_count += 1
            
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                avg_time = sum(process_times) / len(process_times)
                
                # 送信時間だけを表示しています
                print(f"Loop FPS: {fps:.2f} | 送信にかかった時間(Avg): {avg_time:.3f} ms")
                
                frame_count = 0
                process_times = []
                prev_time = curr_time

    except KeyboardInterrupt:
        print("\n[メイン] 測定終了")
    
    finally:
        if ser and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()