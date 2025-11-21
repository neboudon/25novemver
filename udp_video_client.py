import cv2
import numpy as np
import time
import socket

CAMERA_INDEX = 0

#フレーム圧縮関数
def compress_frame(frame, quality):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    result, encimg = cv2.imencode('.jpg', frame, encode_param)
    if result:
        return encimg.tobytes()#配列データをバイトデータに変換して返す
    else:
        return None


def main():
    
    #ソケットの作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    #UDP送信先の設定
    #server_address = ''
    #server_port = ''
    #print(f"送信元アドレス: {server_address}, ポート: {server_port}")
    client_ip = '' #送信先クライアントのIPアドレスを指定
    client_port = 5005 #送信先クライアントのポートを指定
    
    address = (client_ip,client_port) #送信先クライアントのアドレスとポートを指定
    print(  f"送信先アドレス: {address[0]}, ポート: {address[1]}")
    
    #sock.bind((address[0], address[1]))# ソケットを特定のアドレスとポートにバインド
    
    sock.connect(address) #送信先に接続
    
    #カメラの起動
    print("カメラを起動します。")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # 画像サイズを小さくしてデータ量を抑える（推奨）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
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
            
            compressed_frame = compress_frame(frame, quality=50)
            
            if compressed_frame is None:
                print("フレームの圧縮に失敗しました。")
                continue
            
            # UDPの限界サイズを超えているとエラーで落ちるため、送信をスキップします
            if len(compressed_frame) > 65507:
                print(f"データサイズ過大によりスキップ: {len(compressed_frame)} bytes")
                continue
            
            sock.send(compressed_frame)
            #cv2.imshow('Frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    
    finally:    
        cap.release() #カメラを解放
        cv2.destroyAllWindows() #ウィンドウを閉じる
        
        sock.close() #ソケットを閉じる
        
if __name__ == "__main__":
    main()