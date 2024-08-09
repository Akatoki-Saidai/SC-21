import builtins
import time
from smbus2 import SMBus
from bmp280 import BMP280
from bno055 import BNO055
import serial
from micropyGPS import MicropyGPS
from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import LED
import cv2
import numpy as np
from picamera2 import Picamera2


# ※全然ライブラリ化してません


# 少々ここらで(print関数を)オーバーライド
original_print = print

def custom_print(*args, **kwargs):
    # ログファイルへの書き込みを行えるようにする
    with open('/home/jaxai/log/log_test.txt', 'a') as f:
        f.write(' '.join(map(str, args)) + '\n')
    
    original_print(*args, **kwargs)

# print関数をカスタム関数に置き換える
builtins.print = custom_print



# カメラ用関数定義ゾーン
def red_detect(frame):
    # HSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 117, 104])
    hsv_max = np.array([11, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([169, 117, 104])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2


def analyze_red(frame, mask):
        
    camera_order = 4
    # 画像の中にある領域を検出する
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    #画像の中に赤の領域があるときにループ
    if 0 < len(contours):
                    
        # 輪郭群の中の最大の輪郭を取得する-
        biggest_contour = max(contours, key=cv2.contourArea)

        # 最大の領域の外接矩形を取得する
        rect = cv2.boundingRect(biggest_contour)

        # #最大の領域の中心座標を取得する
        center_x = (rect[0] + rect[2] // 2)
        center_y = (rect[1] + rect[3] // 2)

        # 最大の領域の面積を取得する-
        area = cv2.contourArea(biggest_contour)

        # 最大の領域の長方形を表示する
        cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 255), 2)

        # 最大の領域の中心座標を表示する
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

        # 最大の領域の面積を表示する
        cv2.putText(frame, str(area), (rect[0], rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)

        cv2.putText(frame, str(center_x), (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)


        frame_center_x = frame.shape[1] // 2
        # 中心座標のx座標が画像の中心より大きいか小さいか判定
        if area > 10000:
            print("十分近い")
            camera_order = 0

        else:
            if frame_center_x -  50 <= center_x <= frame_center_x + 50:
                print("赤色物体は画像の中心にあります。")#直進
                camera_order = 1
            elif center_x > frame_center_x + 50:
                print("赤色物体は画像の右側にあります。")#右へ
                camera_order = 2
            elif center_x < frame_center_x - 50:
                print("赤色物体は画像の左側にあります。")#左へ
                camera_order = 3

        # red_result = cv2.drawContours(mask, [biggest_contour], -1, (0, 255, 0), 2)

    return camera_order




# mainゾーン

def main():
    
    # ---セットアップゾーン---
    
    # UART(GPS)通信設定(Check pin)
    try:
        uart = serial.Serial('/dev/serial0', 38400, timeout = 10)
    except Exception as e:
        print(f"An error occured in setting serial 0: {e}")

    try:
        #BNOの電源ピンをHighにする
        v_bno = LED(9)
        v_bno.on()

        #BMEの電源ピンをHighにする
        v_bme = LED(27)
        v_bme.on()

        #LEDもつけてみる
        LED_1 = LED(23)
        LED_1.on()
    except Exception as e:
        print(f"An error occured in turnon bmp, bno, led: {e}")


    # モータードライバセットアップ
    try:
        PIN_AIN1 = 18
        PIN_AIN2 = 23
        PIN_BIN1 = 24
        PIN_BIN2 = 13

        dcm_pins = {
            "left_forward": PIN_AIN2,
            "left_backward": PIN_AIN1,
            "right_forward": PIN_BIN2,
            "right_backward": PIN_BIN1,
        }

        factory = PiGPIOFactory()
        motor_left = Motor( forward=dcm_pins["left_forward"],
                            backward=dcm_pins["left_backward"],
                            pin_factory=factory)
        motor_right = Motor( forward=dcm_pins["right_forward"],
                            backward=dcm_pins["right_backward"],
                            pin_factory=factory)
    except Exception as e:
        print(f"An error occured in setting motor_driver: {e}")


    # gpsのセットアップ
    try:
        my_gps = MicropyGPS(9, 'dd')
    except Exception as e:
        print(f"An error occured in setting gps object: {e}")
    
    try:
         tm_last = 0
         tm_interval = 1
    except Exception as e:
            print(f"An error occured in setting serial 0: {e}")


    #bno055のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
            exit()
        time.sleep(1)
        bno.setExternalCrystalUse(True)
    except Exception as e:
        print(f"An error occured in setting bno055 object: {e}")
        

    #bmp280のセットアップ
    try:
        bus = SMBus(1)
        bmp280 = BMP280(i2c_dev=bus)
    except Exception as e:
            print(f"An error occured in setting bmp280 object: {e}")

    # bmp280高度算出用基準気圧取得
    try:
        baseline_values = []
        baseline_size = 20

        for i in range(baseline_size):
            pressure = bmp280.get_pressure()
            baseline_values.append(pressure)
            time.sleep(0.5)
        baseline = sum(baseline_values[:-15]) / len(baseline_values[:-15])
    except Exception as e:
            print(f"An error occured in getting bmp280 data: {e}")

    # カメラセットアップ
    try:
        CameraStart = False
        # picam2 = Picamera2()
        # config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (320, 240)})
        # picam2.configure(config)

    except Exception as e:
        print(f"An error occurred in init camera: {e}")



    # ---繰り返しゾーン---

    while True:

        # UART(GPS)受信データ取得
        try:
            sentence = uart.readline()
        except Exception as e:
                print(f"An error occured in getting data from serial 0: {e}")
                # print(len(sentence))
                # continue

        try:
            # 最高速で正回転 - 1秒
            print("Positive rotation at maximum speed - 1s")
            motor_left.value = 1.0
            motor_right.value = 1.0
            time.sleep(1)
            # 少し遅く正回転 - 1秒
            print("Positive rotation at slightly slow speed - 1s")
            motor_left.value = 0.75
            motor_right.value = 0.75
            time.sleep(1)
            # 遅く正回転 - 2秒
            print("Positive rotation at slow speed - 1s")
            motor_left.value = 0.5
            motor_right.value = 0.5
            time.sleep(1)
            # 停止 - 1秒
            print("stop motor rotation - 1s")
            motor_left.value = 0.0
            motor_right.value = 0.0
            time.sleep(1)
            # 最高速で逆回転 - 1秒
            print("Reverse rotation at maximum speed - 1s")
            motor_left.value = -1.0
            motor_right.value = -1.0
            time.sleep(1)
            # 少し遅く逆回転 - 1秒
            print("Reverse rotation at slightly slow speed - 1s")
            motor_left.value = -0.75
            motor_right.value = -0.75
            time.sleep(1)
            # 遅く逆回転 - 2秒
            print("Reverse rotationat slow speed - 1s")
            motor_left.value = -0.5
            motor_right.value = -0.5
            time.sleep(1)
            # 停止 - 1秒
            print("stop motor rotation - 1s")
            motor_left.value = 0.0
            motor_right.value = 0.0
            time.sleep(1)
        except Exception as e:
            print(f"An error occured in moving motor: {e}")
            # 停止
            motor_left.value = 0.0
            motor_right.value = 0.0


        # GPS緯度経度読み取り
        try:
            if len(sentence) > 0:
                #print("4")
                for x in sentence:
                    if 10 <= x <= 126:
                        #print("5")
                        try:
                            stat = my_gps.update(chr(x))
                        #print("stat:",stat,"x:",x,"chr:",chr(x))
                        #print(chr(x))
                        except Exception as e:
                                print(f"An error occured in updating GPS data: {e}")
                        
                        if stat:
                            #print("6")
                            try:
                                tm = my_gps.timestamp
                                tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])                            
                                if (tm_now - tm_last) >= tm_interval:
                                    latitude, longitude = my_gps.latitude[0], my_gps.longitude[0]
                                    print('=' * 20)
                                    print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                                    print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
                                    tm_last = tm_now
                            except Exception as e:
                                    print(f"An error occured in loading GPS data : {e}")

        except Exception as e:
            print(f"An error occured in reading GPS tm, lat,lon: {e}")


        # bno055データ取得
        try:
            Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
            Mag = bno.getVector(BNO055.VECTOR_MAGNETOMETER)
            Accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
            Accel_all = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
            print("Gyro: ", Gyro)
            print("Mag: ", Mag)
            print("Accel", Accel)
            print("Accel_all", Accel_all)
        except Exception as e:
            print(f"An error occured in reading bno055: {e}")
    
        # bmp280データ取得(基準高度参照)
        try:
            temperature = bmp280.get_temperature()
            pressure = bmp280.get_pressure()
            altitude = bmp280.get_altitude(qnh=baseline)
            print("", temperature)
            print("", pressure)
            print(f"temperture{temperature:05.2f}*C")
            print(f"pressure: {pressure:05.2f}hPa")
            print(f"Relative altitude: {altitude:05.2f} metres")
        except Exception as e:
            print(f"An error occured in reading bmp280: {e}")


        # カメラデータ取得&処理
        # try:
            # フレームを取得
            # if (CameraStart == False):
                # picam2.start()
                # CameraStart = True
            # if (CameraStart == True):
                # frame = picam2.capture_array()
                # 赤色を検出
                # mask = red_detect(frame)
                # 面積のもっとも大きい領域を表示
                # camera_order = analyze_red(frame, mask)
                # 結果表示
                # cv2.imshow("Frame", frame)
                # cv2.imshow("Mask", mask)
                # time.sleep(100)
                #print(len(contours))

                # qキーを押すと終了(手動停止)
                # if cv2.waitKey(25) & 0xFF == ord('q'):
                #     break
        # except Exception as e:
            # print(f"An error occured in processing camera: {e}")




# この.pyファイルがメイン関数の時のみ実行
if __name__ == "__main__":
    while True:
        main()
