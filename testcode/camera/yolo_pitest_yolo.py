import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2
import time

# 同じディレクトリに重みを置く
pt_path = "./SC-21_yolomodel_v1_straight.pt"

# なんかフェーズの変数(近距離フェーズ)
phase = 2
CameraStart = False


def picam_setup(picam):
    #カメラの設定
    try:
        picam = Picamera2()
        config = picam.create_preview_configuration({"format": 'XRGB8888', "size": (416, 416)})
        #config["main"]
        picam.configure(config)

        #カメラの詳細設定(一応設定)
        picam.brightness = 40 #輝度(0～100)
        picam.saturation = 30   #彩度(-100～100)
        picam.ISO = 0 #ISO感度(0～1600, 0は自動)

        picam.shutter_speed = 100000 #シャッター速度(マイクロ秒,0は自動)
        picam.framerate = 10
        picam.exposure_compensation = -20 #露出補正(-25～25)
        picam.exposure_mode = 'auto' #露出モード 
        picam.meter_mode = 'auto' #測光モード
        picam.awb_mode = 'auto' #ホワイトバランス
        picam.awb_gains = (1.0,1.0) #手動AWB調整(0.0～8.0)
        picam.image_effect = 'none' #画像効果
        picam.color_effects = None #カラー効果

        picam.rotation = 0 #回転(0～359）
        picam.hflip = False #水平反転
        picam.vflip = False #垂直反転
        picam.crop = (0.0, 0.0, 1.0, 1.0) #切り抜き(0.0～1.0)
    

    except Exception as e:
        print(f"An unexpected error occurred in init camera: {e}")


picam2 = Picamera2()
picam_setup(picam2)


if (phase == 2 and CameraStart == False):

    picam2.start()
    CameraStart = True

if (CameraStart == True):
    
    # フレームを取得
    frame = picam2.capture_array()

    # モデル読み込み
    model = YOLO(pt_path)

    # 入力画像
    yolo_results = model(frame, save = True, show = True)

    time.sleep(100)
    
    #こっからガチのテスト用
    #print(len(contours))

