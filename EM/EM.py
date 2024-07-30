import time
from smbus2 import SMBus
import serial
from gpiozero import LED
from picamera2 import Picamera2
import math
import numpy as np

# scに使用ライブラリほぼまとめました
from sc import motor
import sc.print_override as override
from sc.camera import Camera
from sc.bmp280 import BMP280
from sc.bno055 import BNO055
from sc.micropyGPS import MicropyGPS
from sc import csv_print as csv
from sc import calc_xy

# mainゾーン

def main():
    
    # ---セットアップゾーン---
    
    # フェーズ，ゴール設定
    try:
        phase = 0
        csv.print('phase', phase)
        goal_latitude = 40.142621667
        csv.print('goal_lat', goal_latitude)
        goal_longtitude = 139.987548333
        csv.print('goal_lon', goal_longtitude)
    except Exception as e:
        print(f"An error occured in initialize phase and goal: {e}")
        csv.print('serious_error', f"An error occured in initialize phase and goal: {e}")

    # 少々ここらで(print関数を)オーバーライド
    # ※printしたものがログファイルにも行きます
    override.printoverride()


    # UART(GPS)通信設定(Check pin)
    try:
        uart = serial.Serial('/dev/serial0', 38400, timeout = 10)
    except Exception as e:
        print(f"An error occured in setting serial 0: {e}")
        csv.print('serious_error', f"An error occured in setting serial 0: {e}")


    try:
        #BNOの電源ピンをHighにする
        v_bno = LED(9)
        v_bno.on()
        #BNOのリセットピンをHighにする
        v_bno_reset = LED(24)
        v_bno_reset.on()

        #BMEの電源ピンをHighにする
        v_bme = LED(27)
        v_bme.on()

        #LEDつけてみる
        LED_1 = LED(23)
        LED_1.on()
    except Exception as e:
        print(f"An error occured in turn on bmp, bno, led: {e}")
        csv.print('error', f"An error occured in turn on bmp, bno, led: {e}")


    # モータードライバセットアップ
    try:
        PIN_AIN1 = 18
        PIN_AIN2 = 23
        PIN_BIN1 = 24
        PIN_BIN2 = 13

        motor_right, motor_left = motor.setup(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2)

    except Exception as e:
        print(f"An error occured in setting motor_driver: {e}")
        csv.print('serious_error', f"An error occured in setting motor_driver: {e}")


    # gpsのセットアップ
    try:
        gnss = MicropyGPS(9, 'dd')
    except Exception as e:
        print(f"An error occured in setting gps object: {e}")
        csv.print('serious_error', f"An error occured in setting gps object: {e}")


    #bno055のセットアップ
    try:
        bno = BNO055()
        if bno.begin() is not True:
            print("Error initializing device")
            pass
        time.sleep(1)
        bno.setExternalCrystalUse(True)
    except Exception as e:
        print(f"An error occured in setting bno055 object: {e}")
        csv.print('serious_error', f"An error occured in setting bno055 object: {e}")
        

    #bmp280のセットアップ
    try:
        bus = SMBus(1)
        bmp = BMP280(i2c_dev=bus)
    except Exception as e:
        print(f"An error occured in setting bmp object: {e}")
        csv.print('serious_error', f"An error occured in setting bmp280 object: {e}")

    # bmp280高度算出用基準気圧取得
    try:
        baseline = bmp.get_baseline()
        print("baseline: ", baseline)
        csv.print('alt_base_press', baseline)
        first_altitude = bmp.get_altitude()
        csv.print('msg', f'first_altitude: {first_altitude}')

    except Exception as e:
        print(f"An error occured in getting bmp data: {e}")
        csv.print('serious_error', f"An error occured in getting bmp280 data: {e}")

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



    # ---繰り返しゾーン---

    while True:

        csv.print('phase', phase)

        # ************************************************** #
        #             待機フェーズ(phase = 0)                #
        # ************************************************** #
        
        if (phase == 0):
            try:                
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
                if (altitude - first_altitude > 10):
                    phase = 1
                    print("Go to falling phase")
                    csv.print('msg', 'Go to falling phase')
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
                    Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
                    # Mag = bno.getVector(BNO055.VECTOR_MAGNETOMETER)
                    Accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    # Accel_all = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
                    print("Gyro: ", Gyro)
                    # print("Mag: ", Mag)
                # 加速度をprint
                    print("Accel", Accel)
                    # print("Accel_all", Accel_all)
                except Exception as e:
                    print(f"An error occured in reading bno055: {e}")
                    csv.print('error', f"An error occured in reading bno055: {e}")

                # z方向の加速度Accel[2]が0，altitudeがbaselineから±3になったら移行 -> SC19を参考に変更
                # 条件式を記述し，フェーズ移行
                #ジャイロを条件式に入れてもいいかもね。不等式の値は適当だからあとで変えておいて。
                if altitude - first_altitude < 3 and sum(abs(Accel)) < 1.5 and sum(abs(Gyro)) < 1.0:
                    time.sleep(0.5)
                    Gyro, Accel = bno.getVector(BNO055.VECTOR_GYROSCOPE), bno.getVector(BNO055.VECTOR_LINEARACCEL)
                    if sum(abs(Accel)) < 1.5 and sum(abs(Gyro)) < 1.0:  # 0.5s後にもう一度判定
                        phase = 2
                        print("Go to long phase")
                        csv.print('msg', "Go to long phase")

                else:
                    pass
    
            except Exception as e:
                print(f"An error occured in falling phase: {e}")
                csv.print('error', f"An error occured in falling phase: {e}")
            
        # z方向の加速度が0，altitudeがbaselineから±3になったら落下フェーズに移行
            


        # ************************************************** #
        #            遠距離フェーズ(phase = 2)               #
        # ************************************************** #
        
        elif (phase == 2):

            try:
                # UART(GPS)受信データ取得
                try:
                    sentence = uart.readline()

                except Exception as e:
                    print(f"An error occured in getting data from serial 0: {e}")
                    csv.print('error', f"An error occured in getting data from serial 0: {e}")


                # GPSの緯度経度取得
                try:
                    if len(sentence) > 0:
                        for x in sentence:
                            if 10 <= x <= 126:
                                try:
                                    stat = gnss.update(chr(x))
                                #print("stat:",stat,"x:",x,"chr:",chr(x))
                                #print(chr(x))
                                except Exception as e:
                                    print(f"An error occured in updating GPS data: {e}")
                                    csv.print('error', f"An error occured in updating GPS data: {e}")
                                
                                if stat:
                                    try:
                                        tm = gnss.timestamp
                                        # tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                                        latitude, longtitude = gnss.latitude[0], gnss.longitude[0]
                                        # print('=' * 20)
                                        print(gnss.date_string(), tm[0], tm[1], int(tm[2]))
                                        print("latitude:", gnss.latitude[0])
                                        print("longitude:", gnss.longitude[0])
                                    except Exception as e:
                                        print(f"An error occured in loading GPS data : {e}")
                                        csv.print('errro', f"An error occured in loading GPS data : {e}")

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
                    # print("Accel", Accel)
                    # print("Accel_all", Accel_all)
                except Exception as e:
                    print(f"An error occured in reading bno055: {e}")
                    csv.print('error', f"An error occured in reading bno055: {e}")


                # 計算過程はcalc_xyに定義
                # ゴールの緯度経度はgoal_latitudeとgoal_longtitude(一番上でフェーズ初期化と一緒に定義)
                try:
                    #1.ゴールの緯度経度をCanSat中心のxy座標で表す。
                    goal_xy = calc_xy.calc_xy(goal_latitude,goal_longtitude,latitude,longtitude)
                    
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
                    North_angle_rad = np.arctan2(Mag[1],Mag[0])
                    cansat_to_goal = calc_xy.Rotation_clockwise_xy(goal_xy,North_angle_rad - math.pi)
                    
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
                        time.sleep(3)

                    if (30 < cansat_to_goal_angle_degree <=135):
                        print("right")
                        motor.rightturn(motor_right, motor_left)
                        time.sleep(1)

                    if (135 <= cansat_to_goal_angle_degree <= 180):
                        print("sharp_right")
                        motor.rightturn(motor_right, motor_left)
                        motor.rightturn(motor_right, motor_left)
                        time.sleep(1)

                    if (225 < cansat_to_goal_angle_degree <= 330):
                        print("left")
                        motor.leftturn(motor_right, motor_left)
                        time.sleep(1) 

                    if (180 < cansat_to_goal_angle_degree < 225):
                        print("sharp_left")
                        motor.leftturn(motor_right, motor_left)
                        motor.leftturn(motor_right, motor_left)
                        time.sleep(1)  

                except Exception as e:
                    print(f"An error occured in calculating goal_xy")
                    csv.print('error', f"An error occured in calculating goal_xy")


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


        # ゴールとのdistanceが5m 以下になったら近距離フェーズに移行

        # ************************************************** #
        #            近距離フェーズ(phase = 3)               #
        # ************************************************** #
       
        elif (phase == 3):

            try:

                ## カメラを起動
                if (CameraStart == False):
                    picam2.start()
                    CameraStart = True

                ## カメラの取得したフレームから赤色を探す
                if (CameraStart == True):
                    frame = picam2.capture_array()
                    # 赤色を検出
                    mask = cam.red_detect(frame)
                    # 赤色検知の結果を取得
                    # analize_redの戻り値は0が見つからない，1が中心，2が右，3が左，4がゴール
                    camera_order = cam.analyze_red(frame, mask)
                    # 結果表示
                    time.sleep(1)
                    #print(len(contours))

                    ## カメラの赤色検知関数の戻り値を参考にしてモーターを動かす

                    try:
                        # 中央にゴールがあるとき一秒前進
                        if (camera_order == 1):
                            # モーターを回転して前進
                            motor.accel(motor_right, motor_left)
                            print("motor: forward -1s")
                            time.sleep(1)  # 1秒進む

                            # モーターの回転を停止
                            motor.brake(motor_right, motor_left)
                            print("motor: brake")
                            time.sleep(1)  # 1秒止まる


                        # 右にゴールがあるとき左に回転
                        elif (camera_order == 2):
                            # モーターを回転させ，CanSatを1秒くらい左回転
                            motor.rightturn(motor_right, motor_left)
                            print("motor: leftturn")
                            time.sleep(1)  # 1秒止まる

                        # 左にゴールがあるとき右に回転
                        elif (camera_order == 3):
                            # モーターを回転させ，CanSatを1秒くらい右回転
                            motor.leftturn(motor_right, motor_left)
                            print("motor: rightturn")
                            time.sleep(1)  # 1秒止まる

                        # ゴールが見つからないとき右に回転
                        elif (camera_order == 0):
                            # モーターを回転させ，CanSatを1秒くらい右回転
                            motor.rightturn(motor_right, motor_left)
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
                            phase = 4
                            # ゴール判定
                            print("Goal Goal Goal")
                            break
                        
                        else:
                            pass
                    except Exception as e:
                        print(f"An error occured in judging goal... (;_;): {e}")
                        csv.print('error', f"An error occured in judging goal...: {e}")



            except Exception as e:
                print(f"An error occured in short phase: {e}")
                csv.print('error', f"An error occured in short phase: {e}")





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
    #     # GPSデータを取得し，今いるlatitude, longtitudeを取得できる

    #     # UART(GPS)受信データ取得
    #     try:
    #         sentence = uart.readline()
            
    #     except Exception as e:
    #             print(f"An error occured in getting data from serial 0: {e}")


    #     # GPS緯度経度読み取り
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
    #                             latitude, longtitude = gnss.latitude[0], gnss.longitude[0]
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
        main()
