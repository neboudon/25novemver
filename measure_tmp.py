import pyrealsense2 as rs
import numpy as np

# カメラの設定
conf = rs.config()
# RGB
conf.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# 距離
conf.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# stream開始
pipe = rs.pipeline()
profile = pipe.start(conf)

cnt = 0

try:
    while True:
        frames = pipe.wait_for_frames()
        
        # frameデータを取得
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # 画像データに変換
        color_image = np.asanyarray(color_frame.get_data())
        # 距離情報をカラースケール画像に変換する
        depth_color_frame = rs.colorizer().colorize(depth_frame)
        depth_image = np.asanyarray(depth_color_frame.get_data())

        #お好みの画像保存処理
        # 画面の中央のピクセル座標を指定
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        target_x = int(width / 2)
        target_y = int(height / 2)
        
        # get_distance(x, y) で (x, y) の距離 [メートル] を取得
        distance = depth_frame.get_distance(target_x, target_y)

        print(f"画面中央 ({target_x}, {target_y}) までの距離: {distance:.2f} メートル")
        
except Exception as e:
    print(f"エラーが発生しました: {e}")
    
finally:
    pipe.stop()
    print("ストリーミングを停止しました。")