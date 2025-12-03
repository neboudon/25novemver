import cv2

def main():
    # カメラデバイスの取得
    # 引数の 0 は接続されている最初のカメラを指します
    # うまくいかない場合は -1 や 1 を試してください
    cap = cv2.VideoCapture(0)

    # カメラが開けたか確認
    if not cap.isOpened():
        print("カメラを開くことができませんでした。接続を確認してください。")
        return

    # 解像度の設定（必要に応じて変更可能）
    # 処理を軽くするために640x480に設定しています
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("カメラを起動しました。'q' キーを押すと終了します。")

    try:
        while True:
            # フレームを1枚キャプチャ
            ret, frame = cap.read()

            # キャプチャに失敗した場合（カメラ切断など）
            if not ret:
                print("映像を取得できませんでした。終了します。")
                break

            # 映像をウィンドウに表示
            cv2.imshow('Camera Feed', frame)

            # キー入力を待機（1ms）
            # 'q' キーが押されたらループを抜ける
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        # Ctrl+C で強制終了された場合の安全策
        pass

    finally:
        # 終了処理
        cap.release()
        cv2.destroyAllWindows()
        print("プログラムを終了しました。")

if __name__ == "__main__":
    main()