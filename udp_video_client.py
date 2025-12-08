import cv2
import socket
import time

# ffmpegで /dev/video42 を指定しているので合わせる
CAMERA_INDEX = 42

# 画質設定 (30くらいが速度と画質のバランス良)
JPEG_QUALITY = 30
SERVER_IP = '192.168.10.222'
SERVER_PORT = 5005

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (SERVER_IP, SERVER_PORT)
    
    print(f"送信先: {address}")
    
    # 起動
    print("カメラを起動します...")
    # V4L2バックエンド指定
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    
    # バッファを最小にして遅延を減らす
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("カメラが開けません。ffmpegが動いていないか、番号(42/0)が違います。")
        return

    print("送信開始。終了は Ctrl+C")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # 映像が来ないときは少し待って再トライ
                time.sleep(0.1)
                continue
            
            # 【高速化】ffmpegでリサイズ済みなので、ここでのresizeは削除しました
            # 【高速化】ラズパイ側での imshow (画面表示) も削除しました

            # 圧縮
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, encimg = cv2.imencode('.jpg', frame, encode_param)
            
            if not result:
                continue

            # 送信
            data = encimg.tobytes()
            if len(data) < 65507:
                sock.sendto(data, address)
            
    except KeyboardInterrupt:
        print("\n終了します。")
    except Exception as e:
        print(f"エラー: {e}")
    finally:
        cap.release()
        sock.close()

if __name__ == "__main__":
    main()