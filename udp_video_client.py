import cv2
import socket
import time

# ================= 設定エリア =================
# ラズパイに直結したカメラは通常 0 です
CAMERA_INDEX = 0

# 送信先（受信側PC）のIPアドレスとポート
SERVER_IP = '192.168.10.222'
SERVER_PORT = 5005

# 送信解像度（ラズパイの負荷を下げるため小さめに設定）
WIDTH = 640
HEIGHT = 480
FPS = 30

# 画質設定 (10-100)
# 30-50くらいが速度と画質のバランスが良いです
JPEG_QUALITY = 35
# ============================================

def main():
    # UDPソケットの作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (SERVER_IP, SERVER_PORT)
    
    print(f"送信先アドレス: {address}")
    
    # カメラの起動
    print("ラズパイカメラを起動します...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # 【重要】ラズパイカメラの設定
    # ハードウェア側でリサイズさせることでCPU負荷を大幅に減らします
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    
    # 遅延対策：バッファサイズを最小にする
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("エラー: カメラが開けません。")
        print("以下を確認してください：")
        print("1. カメラケーブルが正しく接続されているか")
        print("2. (古いOSの場合) raspi-config でLegacy Cameraが有効になっているか")
        return

    print(f"送信開始 ({WIDTH}x{HEIGHT})。停止するには Ctrl+C を押してください。")

    try:
        while True:
            # フレームの読み込み
            ret, frame = cap.read()
            if not ret:
                print("フレーム取得失敗")
                time.sleep(0.1)
                continue
            
            # 【変更点】
            # cap.setで解像度を指定しているので、ここで cv2.resize を呼ぶ必要はありません。
            # 既に 640x480 で取得されています。

            # JPEG圧縮 (必須)
            # 生データだとUDPの制限(65KB)を即座に超えるため、必ず圧縮して送信します
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
                # 圧縮率(JPEG_QUALITY)を下げるか、解像度を下げてください
                print(f"サイズオーバーによりスキップ: {len(data)} bytes")
            
    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        cap.release()
        sock.close()

if __name__ == "__main__":
    main()