import cv2
import socket
import numpy as np

client_ip = ''  # クライアントのIPアドレスを指定
client_port = 5005  # クライアントのポートを指定

def main():
    # ソケットの作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (client_ip, client_port)
    sock.bind(server_address)
    print(f"サーバーはアドレス {server_address[0]}、ポート {server_address[1]} で待機中...")

    while True:
        data, address = sock.recvfrom(65536)  # バッファサイズを大きく設定
        if data:
            # 受信したデータをnumpy配列に変換
            nparr = np.frombuffer(data, np.uint8)
            # JPEG画像としてデコード
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('Received Frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    sock.close()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()