import time
import serial
import threading

SERIAL_PORT = '/dev/ttyS0' 
SERIAL_BAUDRATE = 115200

# ===================================================================
# スレッド0:コマンド送信スレッド 
# ===================================================================

def command_sending(ser: serial.Serial, command: str, lock: threading.Lock):
    if command is None:
        return
    if ser is None or not ser.is_open:
        print("[コマンド送信スレッド]: シリアルポートが開いていません。")
        return
    
    with lock:
        try:
            ser.write(command.encode('utf-8'))
            ser.flush()
        
        except serial.SerialException as e:
            print(f"[コマンド送信スレッド]: シリアル書き込みエラー: {e}")

def main():
    serial_lock = threading.Lock()
    ser = None
    
    frame_count = 0
    process_times = []
    all_process_times = []
    
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"[メイン]: シリアルポート ({SERIAL_PORT}) を開きました。")
    except serial.SerialException as e:
        print(f"[メイン] エラー: シリアルポート ({SERIAL_PORT}) を開けません。{e}")
        return
    
    prev_time = time.time()
    
    try:
        while True:
            direction = "Lw"
            error = 15.0  # 仮のエラー値
            command = f"{direction} {abs(error):.2f}\n"
            if command.lower() == 'exit':
                break
            proc_start = time.perf_counter()
            command_sending(ser, command, serial_lock)
            proc_end = time.perf_counter()
            proc_time_ms = (proc_end - proc_start) * 1000
            all_process_times.append(proc_time_ms)
            frame_count += 1
            
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                recent_avg = sum(all_process_times[-100:]) / len(all_process_times[-100:]) if all_process_times else 0
                print(f"FPS: {fps:.2f} | Avg Proc Time: {recent_avg:.2f}ms | Sent Command: {command.strip()}")
                frame_count = 0
                prev_time = curr_time
    
    except KeyboardInterrupt:
        print("\n[メイン] Keyboard Interrupt.")
    
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print(f"[メイン]: シリアルポート ({SERIAL_PORT}) を閉じました。")
        print("[メイン]: 終了しました。")
        
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

if __name__ == "__main__":
    main()