#
# ファイル名: robot_control.py (画像表示なし・軽量版)
# 役割: メインの制御プログラム。
#       camera_processing から距離を取得し、Picoにシリアルで「偏差」を送信する。
#       CugoSDK_PID.ino (Hコマンドで停止) に対応。
#

import camera_processing  # 1. 画像処理モジュールをインポート
import serial             # 2. シリアル通信モジュールをインポート
import time

# --- 制御パラメータの定義 ---
TARGET_DISTANCE = 0.75  # 目標とする壁との距離 (0.75m)
# ★★★ 修正点 1: 'right' から 'left' に変更 ★★★
SIDE_TO_CHECK = 'left'   # 'right' (右壁) または 'left' (左壁) に接近する
CONTROL_FPS = 15         # カメラのFPS（`camera_processing`と合わせる）

# --- シリアル通信の設定 ---
# (Picoの接続ポートに合わせて修正してください)
SERIAL_PORT = '/dev/ttyS0' # Pi 4のGPIO (TX/RX) を使う場合
# SERIAL_PORT = '/dev/ttyACM0' # USBで接続する場合
BAUDRATE = 115200

def main():
    print("ロボット制御プログラムを開始します。")
    print(f"目標: {SIDE_TO_CHECK}側の壁に {TARGET_DISTANCE:.2f} m まで接近します。")
    print("プログラムを終了するには Ctrl+C を押してください。")
    
    detector = None
    serial_port = None
    
    try:
        # 1. 画像処理クラスのインスタンスを作成
        detector = camera_processing.WallDetector(fps=CONTROL_FPS)
        
        # 2. シリアルポートの初期化
        serial_port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1.0)
        print(f"{SERIAL_PORT} に接続しました。")

        while True:
            # 3. カメラモジュールから「現在の距離」を取得
            
            # ★★★ 修正点 2: 戻り値を2つ受け取る ★★★
            # 戻り値 (distance, color_image) のうち、画像(2番目)は不要なので _ で捨てる
            current_distance, _ = detector.get_frame_and_distance(SIDE_TO_CHECK)

            command = "" # Picoに送るコマンド
            
            # フレーム取得失敗 or 距離測定不能(0.0)
            if current_distance is None:
                print(" [CONTROL] フレーム取得失敗。")
                continue
            
            # 4. Picoに送るコマンドを生成 (C++のロジックに合わせる)
            if current_distance == 0.0:
                command = "N\n" # "Not Found"
                print(" [CONTROL] 壁を検出不能。Picoに'N'を送信。")
            else:
                error = current_distance - TARGET_DISTANCE
                threshold = 0.02 # 2cm

                if abs(error) < threshold:
                    command = "S\n" # "Straight"
                    print(f" [CONTROL] 距離 OK ({current_distance:.2f} m)。'S'を送信。")
                elif error > 0:
                    # 遠すぎる
                    if SIDE_TO_CHECK == 'right':
                        command = f"R {abs(error):.4f}\n"
                        print(f" [CONTROL] 遠い ({current_distance:.2f} m)。'R'を送信。")
                    else:
                        command = f"L {abs(error):.4f}\n"
                        print(f" [CONTROL] 遠い ({current_distance:.2f} m)。'L'を送信。")
                else: # error < 0
                    # 近すぎる
                    if SIDE_TO_CHECK == 'right':
                        command = f"L {abs(error):.4f}\n"
                        print(f" [CONTROL] 近い ({current_distance:.2f} m)。'L'を送信。")
                    else:
                        command = f"R {abs(error):.4f}\n"
                        print(f" [CONTROL] 近い ({current_distance:.2f} m)。'R'を送信。")

            # 5. シリアルポートにコマンドを送信
            if command:
                serial_port.write(command.encode('utf-8'))
            
            # 6. 可視化処理は削除
            # wait_for_frames()が 1/15秒 待機するため、sleepは不要
            
    except serial.SerialException as e:
        print(f"シリアルエラー: {e}")
        print("Picoが接続されているか、ポート名が正しいか確認してください。")
        
    except KeyboardInterrupt:
        # Ctrl+C が押されたらループを抜ける
        print("\nユーザーにより停止リクエスト (Ctrl+C)")

    finally:
        # --- 7. 終了処理 ---
        print("制御ループを終了。")
        if serial_port and serial_port.is_open:
            try:
                # CugoSDK_PID.ino が待機している "H" (停止) コマンドを送信
                print("Picoに 'H' (停止) コマンドを送信します。")
                serial_port.write("H\n".encode('utf-8')) 
                serial_port.close()
                print("シリアルポートを閉じました。")
            except:
                pass # ポートが既に切断されている場合のエラーを無視
        if detector:
            detector.stop()
            print("カメラを停止しました。")

if __name__ == "__main__":
    main()