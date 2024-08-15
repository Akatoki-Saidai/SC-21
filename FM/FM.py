import time
from smbus2 import SMBus
import serial
from gpiozero import LED
from picamera2 import Picamera2
import math
import numpy as np
import cv2

# scに使用ライブラリほぼまとめました
import motor
import print_override as override
from camera import Camera
from bmp280 import BMP280
from bno055 import BNO055
from micropyGPS import MicropyGPS
import csv_print as csv
import calc_xy

# mainゾーン

DEBUG = True

def main():
    
    # ---セットアップゾーン---

    # LEDをセット
    try:
        led_green = LED(27)
        led_green.off()
        led_red = LED(10)
        led_red.off()
    except Exception as e:
        print(f"An error occured in setting LED: {e}")
        csv.print('error', f"An error occured in setting LED: {e}")
    
    # フェーズ，ゴール設定
    try:
        phase = 0
        csv.print('phase', phase)
        goal_lat_lon = (40.14228, 139.98741)
        goal_latitude = goal_lat_lon[0]
        csv.print('goal_lat', goal_latitude)
        goal_longitude = goal_lat_lon[1]
        csv.print('goal_lon', goal_longitude)
        
        # baselineも先に定義
        baseline = 1013.25
        first_altitude = 0
        
    except Exception as e:
        print(f"An error occured in initialize phase and goal: {e}")
        csv.print('serious_error', f"An error occured in initialize phase and goal: {e}")
        led_red.blink(0.5, 0.5, 10, 0)

    # 少々ここらで(print関数を)オーバーライド
    # ※printしたものがログファイルにも行きます
    try:
        override.printoverride()
    except Exception as e:
        print(f"An error occured in overriding print: {e}")
        csv.print('serious_error', f"An error occured in overriding print: {e}")
        led_red.blink(0.5, 0.5, 10, 0)

    # UART(GPS)通信設定(Check pin)
    try:
        uart = serial.Serial('/dev/serial0', 38400, timeout = 10)
    except Exception as e:
        print(f"An error occured in setting serial 0: {e}")
        csv.print('serious_error', f"An error occured in setting serial 0: {e}")
        led_red.blink(0.5, 0.5, 10, 0)


    try:
        #BNOの電源ピンをHighにする
        v_bno = LED(11)
        v_bno.on()
        #BNOのリセットピンをHighにする
        v_bno_reset = LED(24)
        v_bno_reset.on()

        #BMPの電源ピンをHighにする
        v_bme = LED(22)
        v_bme.on()

        # 抵抗を明示的にLowにしておく
        NiCr_PIN = LED(17)
        NiCr_PIN.off()
        
        # wait
        time.sleep(3)


    except Exception as e:
        print(f"An error occured in turn on bmp, bno: {e}")
        csv.print('serious_error', f"An error occured in turn on bmp, bno: {e}")
        led_red.blink(0.5, 0.5, 10, 0)


    # モータードライバセットアップ
    try:
        PIN_AIN1 = 4
        PIN_AIN2 = 23
        PIN_BIN1 = 13
        PIN_BIN2 = 5

        motor_right, motor_left = motor.setup(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2)

    except Exception as e:
        print(f"An error occured in setting motor_driver: {e}")
        csv.print('serious_error', f"An error occured in setting motor_driver: {e}")
        led_red.blink(0.5, 0.5, 10, 0)


    # gpsのセットアップ
    try:
        gnss = MicropyGPS(9, 'dd')
    except Exception as e:
        print(f"An error occured in setting gps object: {e}")
        csv.print('serious_error', f"An error occured in setting gps object: {e}")
        led_red.blink(0.5, 0.5, 10, 0)


    #bno055のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
            pass
        time.sleep(1)
        bno.setExternalCrystalUse(False)
    except Exception as e:
        print(f"An error occured in setting bno055 object: {e}")
        csv.print('serious_error', f"An error occured in setting bno055 object: {e}")
        led_red.blink(0.5, 0.5, 10, 0)
        

    #bmp280のセットアップ
    try:
        bus = SMBus(1)
        bmp = BMP280(i2c_dev=bus)

        # 初めは異常値が出てくるので，空測定
        for i in range(10):
            try:
                bmp.get_temp_pres()
            except Exception as e:
                print(f"An error occurred during empty measurement in BMP: {e}")
                csv.print('msg', f"An error occurred during empty measurement in BMP: {e}")      
    except Exception as e:
        print(f"An error occured in setting bmp object: {e}")
        csv.print('serious_error', f"An error occured in setting bmp280 object: {e}")
        led_red.blink(0.5, 0.5, 10, 0)

    # bmp280高度算出用基準気圧取得
    try:
        baseline = bmp.get_baseline()
        print("baseline: ", baseline)
        # csv.print('alt_base_press', baseline)
        first_altitude = bmp.get_altitude(qnh=baseline)
        csv.print('msg', f'first_altitude: {first_altitude}')

    except Exception as e:
        print(f"An error occured in getting bmp data: {e}")
        csv.print('serious_error', f"An error occured in getting bmp280 data: {e}")
        led_red.blink(0.5, 0.5, 10, 0)

    # カメラセットアップ
    try:
        CameraStart = False
        picam2 = Picamera2()
        config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (320, 240)})
        picam2.configure(config)
        cam = Camera()

    except Exception as e:
        print(f"An error occurred in init camera: {e}")
        csv.print('serious_error', f"An error occurred in init camera: {e}")
        led_red.blink(0.5, 0.5, 10, 0)



    # ---繰り返しゾーン---

    while True:
        try:

            csv.print('phase', phase)

            # ************************************************** #
            #             待機フェーズ(phase = 0)                #
            # ************************************************** #
            
            if (phase == 0):

                try:
                    led_green.on()
                    led_red.off()

                    # bmp280で高度(altitude)を計測
                    try:
                        # temperature = bmp.get_temperature()
                        # pressure = bmp.get_pressure()
                        altitude = bmp.get_altitude(qnh=baseline)
                        # print(f"temperture{temperature:05.2f}*C")
                        # print(f"pressure: {pressure:05.2f}hPa")
                    # 高度をprint
                        print(f"Relative altitude: {altitude:05.2f} metres")
                    except Exception as e:
                        print(f"An error occured in reading bmp: {e}")
                        csv.print('error', f"An error occured in reading bmp: {e}")

                    # bmpの高度の値とbaselineの値(地上の高度)を比較し，その結果で条件分岐
                    # 条件式を記述し，フェーズ移行
                    if (altitude - first_altitude > 30):
                        phase = 1
                        print("Go to falling phase")
                        csv.print('msg', 'Go to falling phase')
                        led_green.blink(0.5, 0.5)
                    else:
                        pass

                except Exception as e:
                    print(f"An error occured in waiting phase: {e}")
                    csv.print('error', f"An error occured in waiting phase: {e}")



            # 高度がある程度(本番は50m，あとで調整するので暫定でお願い)高くなったら落下フェーズに移行
            
            # ************************************************** #
            #             落下フェーズ(phase = 1)                #
            # ************************************************** #
            
            elif (phase == 1):

                try:
                    # led_green.blink(0.5, 0.5, 1000)
                    led_red.off()
                    
                    # bmpの高度(altitude)取得
                    try:
                        # temperature = bmp.get_temperature()
                        # pressure = bmp.get_pressure()
                        altitude = bmp.get_altitude(qnh=baseline)
                        # print(f"temperture{temperature:05.2f}*C")
                        # print(f"pressure: {pressure:05.2f}hPa")
                    # 高度をprint
                        print(f"Relative altitude: {altitude:05.2f} metres")
                    except Exception as e:
                        print(f"An error occured in reading bmp: {e}")
                        csv.print('error', f"An error occured in reading bmp: {e}")

                    # bnoの重力加速度を除いた加速度(Accel)を取得
                    try:
                        Mag = bno.getVector(BNO055.VECTOR_MAGNETOMETER)
                        Accel_all = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
                        euler = bno.getVector(BNO055.VECTOR_EULER)
                        grav = bno.getVector(BNO055.VECTOR_GRAVITY)
                    except Exception as e:
                        print(f"An error occured in reading bno055: {e}")
                        csv.print('error', f"An error occured in reading bno055: {e}")
                    
                    # bnoの重力加速度を除いた加速度(Accel)を取得
                    try:
                        Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                        Accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                        print("Gyro: ", Gyro)
                        print("Accel: ", Accel)
                    except Exception as e:
                        print(f"An error occured in reading bno055: {e}")
                        csv.print('error', f"An error occured in reading bno055: {e}")

                    # z方向の加速度Accel[2]が0，altitudeがbaselineから±3になったら移行 -> SC19を参考に変更
                    # 条件式を記述し，フェーズ移行
                    #ジャイロを条件式に入れてもいいかもね。不等式の値は適当だからあとで変えておいて。
                    if altitude - first_altitude < 3 and sum(abs(Accel_xyz) for Accel_xyz in Accel) < 0.5 and sum(abs(Gyro_xyz) for Gyro_xyz in Gyro) < 0.05:
                        time.sleep(0.5)
                        Gyro, Accel = bno.getVector(BNO055.VECTOR_GYROSCOPE), bno.getVector(BNO055.VECTOR_LINEARACCEL)
                        if sum(abs(Accel_xyz) for Accel_xyz in Accel) < 0.5 and sum(abs(Gyro_xyz) for Gyro_xyz in Gyro) < 0.05:  # 0.5s後にもう一度判定
                            time.sleep(3)
                            if sum(abs(Accel_xyz) for Accel_xyz in Accel) < 0.5 and sum(abs(Gyro_xyz) for Gyro_xyz in Gyro) < 0.05:
                                # led_green.blink(0.5, 0.5, 20)
                                led_red.on()
                                
                                # パラ分離用抵抗起動
                                NiCr_PIN.on()
                                print("NiCr wire turn on")
                                csv.print('msg', "NiCr wire turn on")
                                time.sleep(10)

                                NiCr_PIN.off()
                                print("NiCr wire turn off. Parachute separated")
                                csv.print('msg', "NiCr wire turn off. Parachute separated")

                                if 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2]:
                                    motor_right.value = -1
                                    motor_left.value = -1
                                    time.sleep(5)
                                    motor_right.value = 0
                                    motor_left.value = 0
                                else:
                                    motor.accel(motor_right, motor_left)
                                    time.sleep(5)
                                    motor.brake(motor_right, motor_left)

                                phase = 2
                                print("Go to long phase")
                                csv.print('msg', "Go to long phase")
                                led_green.off()
                            else:
                                csv.print('msg', 'The stationary condition triple check did not pass. Try again.')
                        else:
                            csv.print('msg', 'The stationary condition double check did not pass. Try again.')
                    else:
                        pass
        
                except Exception as e:
                    print(f"An error occured in falling phase: {e}")
                    csv.print('error', f"An error occured in falling phase: {e}")


            # ************************************************** #
            #            遠距離フェーズ(phase = 2)               #
            # ************************************************** #
            
            elif (phase == 2):

                try:
                    led_green.off()
                    led_red.on()

                    # 機体がひっくり返ってたら回る
                    try:
                        accel_start_time = time.time()
                        if 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2]:
                            while 0 < bno.getVector(BNO055.VECTOR_GRAVITY)[2] and time.time()-accel_start_time < 5:
                                print('muki_hantai')
                                csv.print('warning', 'muki_hantai')
                                motor.accel(motor_right, motor_left)
                                time.sleep(0.5)
                            else:
                                if time.time()-accel_start_time >= 5:
                                    # 5秒以内に元の向きに戻らなかった場合
                                    motor.rightturn(motor_right, motor_left)
                                    motor.leftturn(motor_right, motor_left)
                                    continue
                                else:
                                    print('muki_naotta')
                                    csv.print('msg', 'muki_naotta')
                                    motor.brake(motor_right, motor_left)
                    except Exception as e:
                        print(f"An error occured while changing the orientation: {e}")
                        csv.print('error', f"An error occured while changing the orientation: {e}")
                    
                    # UART(GPS)受信データ，GPSの緯度経度取得
                    try:
                        sentence_all = uart.read(uart.in_waiting).decode('utf-8')
                        print("GPS data received")
                        
                        sentence_list = sentence_all.split('\n')

                        for sentence in sentence_list[-11:-2]:
                            if DEBUG:
                                csv.print('nmea', sentence)
                            for x in sentence:
                                if 10 <= ord(x) <= 126:
                                    try:
                                        stat = gnss.update(x)
                                        #print("stat:",stat,"x:",x,"chr:",chr(x))
                                        #print(chr(x))
                                    except Exception as e:
                                        print(f"An error occured in updating GPS data: {e}")
                                        csv.print('error', f"An error occured in updating GPS data: {e}")
                                    
                                    try:
                                        if stat:
                                            tm = gnss.timestamp
                                            # tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                                            latitude, longitude = gnss.latitude[0], gnss.longitude[0]
                                            print("latitude[1]:",gnss.latitude[1])
                                            print("longitude[1]:",gnss.longitude[1])
                                            if gnss.latitude[1] == "S":
                                                latitude = -1 * latitude
                                            if gnss.longitude[1] == "W":
                                                lomgtitude = -1 * longitude
                                            # print('=' * 20)
                                            print(gnss.date_string(), tm[0], tm[1], int(tm[2]))
                                            print("latitude:", latitude)
                                            print("longitude:", longitude)
                                    except Exception as e:
                                        print(f"An error occured in loading GPS data : {e}")
                                        csv.print('error', f"An error occured in loading GPS data : {e}")
                                    
                    except Exception as e:
                        print(f"An error occured in reading GPS tm, lat,lon: {e}")
                        csv.print('error', f"An error occured in reading GPS tm, lat,lon: {e}")


                    # bno055地磁気Magを取得
                    try:
                        # Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                        Mag = bno.getVector(BNO055.VECTOR_MAGNETOMETER)
                        # Accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                        # Accel_all = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
                        # print("Gyro: ", Gyro)
                        print("Mag: ", Mag)
                        # print("Accel: ", Accel)
                        # print("Accel_all: ", Accel_all)
                    except Exception as e:
                        print(f"An error occured in reading bno055: {e}")
                        csv.print('error', f"An error occured in reading bno055: {e}")


                    # 計算過程はcalc_xyに定義
                    # ゴールの緯度経度はgoal_latitudeとgoal_longitude(一番上でフェーズ初期化と一緒に定義)
                    try:
                        # まず圧倒的に日本の外だったらやり直し
                        if longitude:
                            if (longitude <= 130) or (longitude >= 150):
                                print(f'GNSS measurement value is invalid. latitude: {latitude}')
                                csv.print('error', f'GNSS measurement value is invalid. latitude: {latitude}')
                                continue
                        if latitude:
                            if (latitude <= 30) or (latitude >= 50):
                                print(f'GNSS measurement value is invalid. longitude: {longitude}')
                                csv.print('error', f'GNSS measurement value is invalid. longitude: {longitude}')
                                continue

                        #1.ゴールの緯度経度をCanSat中心のxy座標で表す。
                        goal_xy = calc_xy.calc_xy(goal_latitude,goal_longitude,latitude,longitude)
                        
                        #2.緯度経度→→→ゴールと機体の距離を求める
                        print("goal xy_coordinate: ", goal_xy)                    
                        csv.print('goal_relative', goal_xy)
                        # print(goal_xy[0])

                        cansat_to_goal_y_sq = (goal_xy[1])**2
                        cansat_to_goal_x_sq = (goal_xy[0])**2
                        distance = np.sqrt(cansat_to_goal_x_sq + cansat_to_goal_y_sq)
                        csv.print('goal_distance', distance)

                        #3.機体の正面と北の向きの関係＋北の向きとゴールの向きの関係→→→機体の正面とゴールの向きの関係を求める
                        #やってることとしては東西南北の基底→CanSatの基底に座標変換するために回転行列を使ってる感じ
                        #North_angle_rad - math.piは、平面直交座標のx軸(西)と北の向きを表すときのx軸(機体の正面)が何度ずれているかを表している
                        North_angle_rad = np.arctan2(-1 * Mag[1],Mag[0])
                        cansat_to_goal = calc_xy.Rotation_clockwise_xy(goal_xy,North_angle_rad)
                        
                        #4.CanSatの正面とゴールの向きの関係を角度で表現している(radian→degreeの変換も行う)。ただし、角度の定義域は(0<=degree<=360)。正面は0と360で真後ろが180。
                        cansat_to_goal_angle = np.arctan2(cansat_to_goal[1],cansat_to_goal[0])
                        cansat_to_goal_angle_degree = math.degrees(cansat_to_goal_angle) + 180
                        csv.print('goal_relative_angle_rad', cansat_to_goal_angle)
                        
                        #5.機体の正面とゴールの向きの関係から、右に曲がるか、左に曲がるか、正面に進むか判断する
                        print("cansat to goal angle [degree]: ", cansat_to_goal_angle_degree)
                        print("cansat to goal distance [m]: ", distance)
                        
                        if (cansat_to_goal_angle_degree < 30) or (330 < cansat_to_goal_angle_degree):
                            print("forward")
                            motor.accel(motor_right, motor_left)
                            time.sleep(1)

                            # スタックチェック
                            try:                                
                                is_stacking = 1
                                for i in range(5):
                                    Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                                    gyro_xyz = abs(Gyro[0]) + abs(Gyro[1]) + abs(Gyro[2])
                                    is_stacking = is_stacking and (gyro_xyz < 0.75)
                                    time.sleep(0.2)
                                if is_stacking:
                                    led_green.on()
                                    led_red.blink(0.5, 0.5, 10)
                                    print('stacking now!')
                                    csv.print('warning', 'stacking now!')
                                    motor.rightturn(motor_right, motor_left)
                                    motor.accel(motor_right, motor_left)
                                    time.sleep(1)
                                    motor.brake(motor_right, motor_left)
                                    motor.leftturn(motor_right, motor_left)
                                    motor.accel(motor_right, motor_left)
                                    time.sleep(1)
                                    motor.brake(motor_right, motor_left)
                                    led_green.off()
                                    led_red.on()
                            except Exception as e:
                                print(f"An error occured in stack check: {e}")
                                csv.print('error', f"An error occured in stack check: {e}")

                            motor.brake(motor_right, motor_left)

                        if (30 < cansat_to_goal_angle_degree <=135):
                            print("right")
                            motor.right_angle(bno, cansat_to_goal_angle_degree, motor_right, motor_left)
                            time.sleep(1)

                        if (135 <= cansat_to_goal_angle_degree <= 180):
                            print("sharp_right")
                            motor.right_angle(bno, cansat_to_goal_angle_degree, motor_right, motor_left)                            
                            time.sleep(1)

                        if (225 < cansat_to_goal_angle_degree <= 330):
                            print("left")
                            cansat_to_goal_angle_degree = (-1*cansat_to_goal_angle_degree) + 360
                            motor.left_angle(bno, cansat_to_goal_angle_degree, motor_right, motor_left)                            
                            time.sleep(1) 

                        if (180 < cansat_to_goal_angle_degree < 225):
                            print("sharp_left")
                            cansat_to_goal_angle_degree = (-1*cansat_to_goal_angle_degree) + 360
                            motor.left_angle(bno, cansat_to_goal_angle_degree, motor_right, motor_left)
                            time.sleep(1)  

                    except Exception as e:
                        print(f"An error occured in calculating goal_xy: {e}")
                        csv.print('error', f"An error occured in calculating goal_xy: {e}")


                    # ゴールとの距離が5m(10m?)で近距離フェーズに移行
                    # 条件式を記述し，フェーズ移行
                    try:
                        if (distance <= 5):
                            phase = 3
                            
                            motor.brake(motor_right, motor_left)
                            print("Go to short phase")
                            csv.print('msg', "Go to short phase")

                        else:
                            pass
                    except Exception as e:
                        print(f"An error occured in judging transition to short phase, {e}")
                        csv.print('error', f"An error occured in judging transition to short phase, {e}")

                except Exception as e:
                    print(f"An error occured in long phase: {e}")
                    csv.print('error', f"An error occured in long phase: {e}")


            # ゴールとのdistanceが5m以下になったら近距離フェーズに移行

            # ************************************************** #
            #            近距離フェーズ(phase = 3)               #
            # ************************************************** #
        
            elif (phase == 3):

                try:
                    led_red.on()
                    led_green.on()

                    try:
                        bno.getVector(BNO055.VECTOR_MAGNETOMETER)
                        bno.getVector(BNO055.VECTOR_GYROSCOPE)
                        bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    except Exception as e:
                        print(f"An error occured in reading bno055: {e}")
                        csv.print('error', f"An error occured in reading bno055: {e}")

                    ## カメラを起動
                    if (CameraStart == False):
                        picam2.start()
                        CameraStart = True

                    # 機体が前に傾き過ぎていたら前進して修正
                    grav = bno.getVector(BNO055.VECTOR_GRAVITY)
                    if 6 < grav[0] or 0 < grav[2]:
                        motor.accel(motor_right, motor_left)
                        time.sleep(0.5)
                        motor.brake(motor_right, motor_left)

                    ## カメラの取得したフレームから赤色を探す
                    if (CameraStart == True):
                        frame = picam2.capture_array()
                        # 赤色を検出
                        mask = cam.red_detect(frame)
                        # 赤色検知の結果を取得
                        # analize_redの戻り値は0が見つからない，1が中心，2が右，3が左，4がゴール
                        frame, camera_order = cam.analyze_red(frame, mask)
                        # 結果表示
                        cv2.imshow('kekka', frame)
                        time.sleep(1)
                        #print(len(contours))

                        if cv2.waitKey(25) & 0xFF == ord('q'):
                            cv2.destroyAllWindows()
                            print('q interrupted direction by camera')
                            csv.print('msg', 'q interrupted direction by camera')
                            continue

                        ## カメラの赤色検知関数の戻り値を参考にしてモーターを動かす

                        try:
                            # 中央にゴールがあるとき一秒前進
                            if (camera_order == 1):
                                # モーターを回転して前進
                                motor.accel(motor_right, motor_left)
                                print("forward")
                                time.sleep(1)  # 1秒進む

                                # スタックチェック
                                try:                                
                                    is_stacking = 1
                                    for i in range(5):
                                        Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                                        gyro_xyz = abs(Gyro[0]) + abs(Gyro[1]) + abs(Gyro[2])
                                        is_stacking = is_stacking and (gyro_xyz < 0.75)
                                        time.sleep(0.1)
                                    if is_stacking:
                                        led_green.on()
                                        led_red.blink(0.5, 0.5, 10)
                                        print('stacking now!')
                                        csv.print('warning', 'stacking now!')
                                        motor.rightturn(motor_right, motor_left)
                                        motor.accel(motor_right, motor_left)
                                        time.sleep(1)
                                        motor.brake(motor_right, motor_left)
                                        motor.leftturn(motor_right, motor_left)
                                        motor.accel(motor_right, motor_left)
                                        time.sleep(1)
                                        motor.brake(motor_right, motor_left)
                                        led_green.off()
                                        led_red.on()
                                except Exception as e:
                                    print(f"An error occured in stack check: {e}")
                                    csv.print('error', f"An error occured in stack check: {e}")

                                # モーターの回転を停止
                                motor.brake(motor_right, motor_left)
                                print("brake")
                                time.sleep(1)  # 1秒止まる


                            # 右にゴールがあるとき左に回転
                            elif (camera_order == 2):
                                # モーターを回転させ，CanSatを1秒くらい左回転
                                motor.right_angle(bno, 30, motor_right, motor_left)
                                # motor.rightturn(motor_right, motor_left)
                                print("rightturn")
                                time.sleep(1)  # 1秒止まる

                            # 左にゴールがあるとき右に回転
                            elif (camera_order == 3):
                                # モーターを回転させ，CanSatを1秒くらい右回転
                                motor.left_angle(bno, 30, motor_right, motor_left)
                                # motor.leftturn(motor_right, motor_left)
                                print("leftturn")
                                time.sleep(1)  # 1秒止まる

                            # ゴールが見つからないとき右に回転
                            elif (camera_order == 0):
                                # モーターを回転させ，CanSatを1秒くらい右回転
                                motor.right_angle(bno, 30, motor_right, motor_left)
                                # motor.rightturn(motor_right, motor_left)
                                print("motor: rightturn")
                                time.sleep(1)  # 1秒止まる

                        except Exception as e:
                            print(f"An error occured in moving motor: {e}")
                            # モーターを強制停止
                            motor_left.value = 0.0
                            motor_right.value = 0.0


                            ## 赤色検知関数の戻り値を参考にフェーズ移行
                            ## 条件式を記述し，whileループを抜けてゴール判定
                            #ゴールについたらフェーズ4に移行
                        try:
                            if (camera_order == 4):
                                motor.accel(motor_right, motor_left)
                                time.sleep(0.75)
                                motor.brake(motor_right, motor_left)
                                
                                # cv2.destroyAllWindows()
                                phase = 4
                                csv.print('phase', phase)
                                # ゴール判定
                                print("Goal Goal Goal")
                                csv.print('msg', 'Goal')

                                led_green.blink(0.5, 0.5)
                                led_red.blink(0.5, 0.5)
                            
                            else:
                                pass
                        except Exception as e:
                            print(f"An error occured in judging goal... (;_;): {e}")
                            csv.print('error', f"An error occured in judging goal...: {e}")


                except Exception as e:
                    print(f"An error occured in short phase: {e}")
                    csv.print('error', f"An error occured in short phase: {e}")
            

            # カメラで取得した画像内の赤色が大きかったらゴールフェーズに移行

            # ************************************************** #
            #            ゴールフェーズ(phase = 4)               #
            # ************************************************** #

            elif phase == 4:
                try:
                    # led_green.blink(0.5, 0.5, 100)
                    # led_red.blink(0.5, 0.5, 100)
                    motor_left.value = 0.0
                    motor_right.value = 0.0

                    # 待っている間にいろいろデータを取得
                    bmp.get_altitude(qnh=baseline)
                    bno.getVector(BNO055.VECTOR_MAGNETOMETER)
                    bno.getVector(BNO055.VECTOR_GYROSCOPE)
                    bno.getVector(BNO055.VECTOR_EULER)
                    bno.getVector(BNO055.VECTOR_GRAVITY)
                    bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    bno.getVector(BNO055.VECTOR_ACCELEROMETER)
                    frame = picam2.capture_array()
                    mask = cam.red_detect(frame)
                    frame, _ = cam.analyze_red(frame, mask)
                    cv2.imshow("after goal", frame)
                    if cv2.waitKey(0):
                        pass
                except Exception as e:
                    print(f"An error occured in goal phase: {e}")
                    csv.print('error', f"An error occured in goal phase: {e}")
        except Exception as e:
            print(f"An error occured in main loop: {e}")
            csv.print('serious_error', f"An error occured in main loop: {e}")






    #     #////////////////////////////////////////////////////////////////////
    #     # 以下記述コード(各フェーズ内のtryに適切なものをコピペしてください)


    #     # "モーター"を使うコード
    #     # これらを使ってゴールの向きに回転し，ゴールへ前進する
    #     # モーターの動き部分の二行とtry，exceptをコピペ
    #     try:
    #         # モーターを回転して前進
    #         motor.accel(motor_right, motor_left)
    #         print("motor: forward -1s")
    #         time.sleep(1)  # 何秒進むか

    #         # モーターの回転を停止
    #         motor.brake(motor_right, motor_left)
    #         print("motor: brake")
    #         time.sleep(1)  # 何秒進むか

    #         # モーターを回転させ，CanSatを1秒くらい右に曲がるように回転
    #         motor.rightturn(motor_right, motor_left)
    #         print("motor: rightturn")

    #         # モーターを回転させ，CanSatを1秒くらい左に曲がるように回転
    #         motor.leftturn(motor_right, motor_left)
    #         print("motor: leftturn")



    #         time.sleep(1)

    #     except Exception as e:
    #         print(f"An error occured in moving motor: {e}")
    #         # モーターを強制停止
    #         motor_left.value = 0.0
    #         motor_right.value = 0.0



    #     # "GPS"を使うコード
    #     # GPSを使う際は以下のコードを使用する
    #     # GPSデータを取得し，今いるlatitude, longitudeを取得できる

    #     # UART(GPS)受信データ取得(旧版)
    #     try:
    #         sentence = uart.readline()
            
    #     except Exception as e:
    #             print(f"An error occured in getting data from serial 0: {e}")


    #     # GPS緯度経度読み取り(旧版)
    #     try:
    #         if len(sentence) > 0:
    #             for x in sentence:
    #                 if 10 <= x <= 126:
    #                     try:
    #                         stat = gnss.update(chr(x))
    #                     #print("stat:",stat,"x:",x,"chr:",chr(x))
    #                     #print(chr(x))
    #                     except Exception as e:
    #                             print(f"An error occured in updating GPS data: {e}")
                        
    #                     if stat:
    #                         try:
    #                             tm = gnss.timestamp
    #                             # tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
    #                             latitude, longitude = gnss.latitude[0], gnss.longitude[0]
    #                             # print('=' * 20)
    #                             print(gnss.date_string(), tm[0], tm[1], int(tm[2]))
    #                             print("latitude:", gnss.latitude[0])
    #                             print("longitude:", gnss.longitude[0])
    #                         except Exception as e:
    #                                 print(f"An error occured in loading GPS data : {e}")

    #     except Exception as e:
    #         print(f"An error occured in reading GPS tm, lat,lon: {e}")


    #     # "BNO055(9軸)"を使うコード
    #     # 地磁気Mag，ジャイロGyro，重力加速度を除く加速度Accel，除かない加速度Accel＿あｌｌ)を取得できる"
    #     # bno055データ取得
    #     try:
    #         Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
    #         Mag = bno.getVector(BNO055.VECTOR_MAGNETOMETER)
    #         Accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
    #         Accel_all = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
    #         print("Gyro: ", Gyro)
    #         print("Mag: ", Mag)
    #         print("Accel", Accel)
    #         print("Accel_all", Accel_all)
    #     except Exception as e:
    #         print(f"An error occured in reading bno055: {e}")
    


    #     # "BMP280(温度，気圧センサ)"を使うコード
    #     # 温度temperture，気圧pressure，相対高度altitudeを取得するコード
    #     # bmp280データ取得(基準高度参照)
    #     try:
    #         temperature = bmp.get_temperature()
    #         pressure = bmp.get_pressure()
    #         altitude = bmp.get_altitude(qnh=baseline)
    #         print(f"temperture{temperature:05.2f}*C")
    #         print(f"pressure: {pressure:05.2f}hPa")
    #         print(f"Relative altitude: {altitude:05.2f} metres")
    #     except Exception as e:
    #         print(f"An error occured in reading bmp: {e}")


    #     # "カメラ"を使うコード
    #     # カメラデータ取得&処理
    #     try:
    #         # フレームを取得
    #         if (CameraStart == False):
    #             picam2.start()
    #             CameraStart = True
    #         if (CameraStart == True):
    #             frame = picam2.capture_array()
    #             # 赤色を検出
    #             mask = cam.red_detect(frame)
    #             # 赤色検知の結果を取得
    #             # analize_redの戻り値は0が見つからない，1が中心，2が右，3が左，4がゴール，
    #             camera_order = cam.analyze_red(frame, mask)
    #             # 結果表示
    #             time.sleep(0.5)
    #             #print(len(contours))

    #     except Exception as e:
    #         print(f"An error occured in processing camera: {e}")

    # # csvwrite.write()
    # # csvの書き込みは準備中




# この.pyファイルがメイン関数の時のみ実行
if __name__ == "__main__":
    while True:
        try:
            main()
        except Exception as e:
            print(f'An error occured in main func: {e}')
            csv.print('serious_error', 'An error occured in main func')
