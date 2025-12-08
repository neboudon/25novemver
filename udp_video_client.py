import cv2
import numpy as np
import time
import socket

# ここを修正：ffmpeg経由の場合は V4L2 バックエンドを明示したほうが安定します
# もしエラーが出る場合は cv2.CAP_V4L2 を削除して 0 だけにしてください
CAMERA_INDEX = 0

# 送信したいサイズを指定
SEND_WIDTH = 640
SEND_HEIGHT = 480

def compress_frame(frame, quality):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    result, encimg = cv2.imencode('.jpg', frame, encode_param)
    if result:
        return encimg.tobytes()
    else:
        return None

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 送信先設定（環境に合わせて変更してください）
    client_ip = '192.168.10.222' 
    client_port = 5005 
    address = (client_ip, client_port)
    
    print(f"送信先アドレス: {address[0]}, ポート: {address[1]}")
    
    # UDP送信なので connect は必須ではありませんが、しておくと send だけ書けるようになります
    sock.connect(address) 
    
    print("カメラを起動します。")
    # V4L2バックエンドを明示的に指定（推奨）
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("カメラを開けません。")
        return
    print("カメラ起動完了")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("フレームを取得できません。")
                break
            
            # 【重要】ここで強制的にリサイズします
            # cap.set が効かないため、読み込んだ画像を小さく加工します
            frame = cv2.resize(frame, (SEND_WIDTH, SEND_HEIGHT))
            
            # 画質設定（サイズが大きい場合はここを 30 くらいまで下げると軽くなります）
            compressed_frame = compress_frame(frame, quality=50)
            
            if compressed_frame is None:
                continue
            
            # データサイズチェック
            data_size = len(compressed_frame)
            if data_size > 65507:
                print(f"サイズ過大によりスキップ: {data_size} bytes (画質を下げてください)")
                continue
            
            sock.send(compressed_frame)
            
            # 送信側の画面にも表示（不要ならコメントアウト）
            cv2.imshow('Sender Frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    
    finally:    
        cap.release()
        cv2.destroyAllWindows()
        sock.close()
        
if __name__ == "__main__":
    main()