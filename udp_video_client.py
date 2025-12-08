import cv2
import socket
import time

# ================= 設定エリア =================
# ffmpegで /dev/video42 を指定した場合は 42、video0なら 0
CAMERA_INDEX = 42 

# 送信先（受信側PC）のIPアドレスとポート
SERVER_IP = '192.168.10.222'
SERVER_PORT = 5005

# 画質設定 (10-100)
# 30くらいが速度と画質のバランスが良いです
JPEG_QUALITY = 30
# ============================================

def main():
    # UDPソケットの作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (SERVER_IP, SERVER_PORT)
    
    print(f"送信先アドレス: {address}")
    
    # カメラの起動
    print("カメラを起動します...")
    # Linux(ラズパイ)での動作を安定させるために CAP_V4L2 を指定
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    
    # 【重要】遅延を減らすため、バッファサイズを最小にする
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("エラー: カメラが開けません。")
        print("ffmpegが実行中か、デバイス番号(42/0)が合っているか確認してください。")
        return

    print("送信開始。停止するには Ctrl+C を押してください。")

    try:
        while True:
            # フレームの読み込み
            ret, frame = cap.read()
            if not ret:
                # 映像が来ていない場合は少し待機して再試行
                time.sleep(0.1)
                continue
            
            # 【補足】
            # 手動実行のffmpegコマンドで「-vf scale=640:480」をつけていれば
            # ここでのリサイズは不要です（CPU節約のため削除済み）。
            # もしffmpegでリサイズしていない場合は、以下のコメントを外してください。
            # frame = cv2.resize(frame, (640, 480))

            # JPEG圧縮
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, encimg = cv2.imencode('.jpg', frame, encode_param)
            
            if not result:
                continue

            # バイトデータに変換
            data = encimg.tobytes()

            # UDPのパケットサイズ制限チェック (約65KB)
            if len(data) < 65507:
                sock.sendto(data, address)
            else:
                print(f"サイズオーバーによりスキップ: {len(data)} bytes")
            
            # 【重要】ラズパイ側での画面表示(imshow)は重くなるので行いません

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        cap.release()
        sock.close()

if __name__ == "__main__":
    main()