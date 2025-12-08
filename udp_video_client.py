import cv2
import numpy as np
import time
import socket

# ffmpegで /dev/video0 に流している場合
CAMERA_INDEX = 0

def compress_frame(frame, quality):
    # 画質設定（30くらいが速度と画質のバランスが良いです）
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    result, encimg = cv2.imencode('.jpg', frame, encode_param)
    if result:
        return encimg.tobytes()
    else:
        return None

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 相手のIPアドレス
    client_ip = '192.168.10.222' 
    client_port = 5005 
    address = (client_ip, client_port)
    
    print(f"送信先: {address}")
    sock.connect(address) 
    
    print("カメラを起動します（ffmpegが動いているか確認してください）")
    # V4L2指定
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    
    # バッファサイズを最小にする（遅延対策）
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("カメラを開けません。")
        return
    print("送信開始。停止するには Ctrl+C を押してください。")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("フレーム取得失敗")
                time.sleep(0.1) # エラー時は少し休む
                continue
            
            # 【変更点】ffmpeg側でリサイズしているので、Pythonでの resize は削除！
            # どうしてもサイズ変更が必要な場合のみ以下を使う
            # frame = cv2.resize(frame, (640, 480))
            
            # 画質を30に設定
            compressed_frame = compress_frame(frame, quality=30)
            
            if compressed_frame is None:
                continue
            
            # サイズチェック
            if len(compressed_frame) > 65507:
                print("サイズオーバー")
                continue
            
            sock.send(compressed_frame)
            
            # 【変更点】ラズパイ側での表示（imshow）は行わない！
            # これが一番重い原因です。
            
    except KeyboardInterrupt:
        print("\n停止しました。")
    except Exception as e:
        print(f"エラー: {e}")
    
    finally:    
        cap.release()
        sock.close()
        
if __name__ == "__main__":
    main()