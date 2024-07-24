import time
from smbus2 import SMBus
import serial
from gpiozero import LED
from picamera2 import Picamera2

# scに使用ライブラリほぼまとめました
from sc import motor
import sc.print_override as override
from sc.camera import Camera
from sc.bmp280 import BMP280
from sc.bno055 import BNO055
from sc.micropyGPS import MicropyGPS
from sc.csv import csvwrite as csvwrite


# mainゾーン

def main():
    
    # ---セットアップゾーン---
    
    # フェーズ，ゴール設定
    try:
        phase = 0
        goal_latitudee = 40.142621667
        goal_longtitude = 139.987548333
    except Exception as e:
        print(f"An error occured in initialize phase and goal: {e}")

    # 少々ここらで(print関数を)オーバーライド
    # ※printしたものがログファイルにも行きます
    override.printoverride()


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

        #LEDつけてみる
        LED_1 = LED(23)
        LED_1.on()
    except Exception as e:
        print(f"An error occured in turn on bmp, bno, led: {e}")


    # モータードライバセットアップ
    try:
        PIN_AIN1 = 18
        PIN_AIN2 = 23
        PIN_BIN1 = 24
        PIN_BIN2 = 13

        motor_right, motor_left = motor.setup(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2)

    except Exception as e:
        print(f"An error occured in setting motor_driver: {e}")


    # gpsのセットアップ
    try:
        gnss = MicropyGPS(9, 'dd')
    except Exception as e:
        print(f"An error occured in setting gps object: {e}")


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
        

    #bmp280のセットアップ
    try:
        bus = SMBus(1)
        bmp = BMP280(i2c_dev=bus)
    except Exception as e:
            print(f"An error occured in setting bmp object: {e}")

    # bmp280高度算出用基準気圧取得
    try:
        baseline = bmp.get_baseline()
        print("baseline: ", baseline)

    except Exception as e:
        print(f"An error occured in getting bmp data: {e}")

    # カメラセットアップ
    try:
        CameraStart = False
        picam2 = Picamera2()
        config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (320, 240)})
        picam2.configure(config)
        cam = Camera()

    except Exception as e:
        print(f"An error occurred in init camera: {e}")



    # ---繰り返しゾーン---

    while True:

        # ************************************************** #
        #             待機フェーズ(phase = 0)                #
        # ************************************************** #
        
        if (phase == 0):
            try:
                # ここにコードを記述(担当：菅原)
                
                # bmp280で高度(altitude)を計測

                # 高度をprint


            except Exception as e:
                print(f"An error occured in waiting phase: {e}")


            # bmpの高度の値とbaselineの値(地上の高度)を比較し，その結果で条件分岐
            # 条件式を記述し，フェーズ移行
            if ():
                phase = 1
                print("Go to falling phase")

            else:
                pass


        # 高度がある程度(本番は50m，あとで調整するので暫定でお願い)高くなったら落下フェーズに移行
        
        # ************************************************** #
        #             落下フェーズ(phase = 1)                #
        # ************************************************** #
        
        elif (phase == 1):

            try:
                # ここにコードを記述(担当：)

                # bmpの高度(altitude)取得
                
                # 高度をprint

                # bnoの重力加速度を除いた加速度(Accel)を取得

                # 加速度をprint


            except Exception as e:
                print(f"An error occured in waiting phase: {e}")


            # z方向の加速度Accel[2]が0，altitudeがbaselineから±3になったら移行
            # 条件式を記述し，フェーズ移行
            if ():
                phase = 2
                print("Go to long phase")

            else:
                pass

        # z方向の加速度が0，altitudeがbaselineから±3になったら落下フェーズに移行

        # ************************************************** #
        #            遠距離フェーズ(phase = 2)               #
        # ************************************************** #
        
        elif (phase == 2):

            try:
                # ここにコードを記述(担当：溝渕)
                # ゴールの緯度経度はgoal_latitudeとgoal_longtitude

                # UARTでGPSのデータ受信

                # GPSの緯度経度取得

                # 地磁気Magを取得


                # ゴールの緯度経度，GPSの緯度経度，地磁気からゴートの角度を算出
                
                # ゴールとの角度の結果を参考にしてモーターを動かす
                # 計算過程は吉澤より粕田に聞いた方が分かりやすいかも


            except Exception as e:
                print(f"An error occured in waiting phase: {e}")




            # ゴールとの距離が5m(10m?)で近距離フェーズに移行
            # 条件式を記述し，フェーズ移行
            if ():
                phase = 3
                print("Go to short phase")

            else:
                pass

        # ゴールとのdistanceが5m 以下になったら近距離フェーズに移行

        # ************************************************** #
        #            近距離フェーズ(phase = 3)               #
        # ************************************************** #
        elif (phase == 3):
            
            try:
                # ここにコードを記述(担当：田中)

                # カメラを起動

                # カメラの取得したフレームから赤色を探す

                # カメラの赤色検知関数の戻り値を参考にしてもーターを動かす



            except Exception as e:
                print(f"An error occured in waiting phase: {e}")


            # 赤色検知関数の戻り値を参考にフェーズ移行
            # 条件式を記述し，whileループを抜けてゴール判定
            if ():
                phase = 4
                # ゴール判定
                try:
                    print("Goal Goal Goal")

                except Exception as e:
                    print(f"An error occured in judging goal... (;_;): {e}")

                break

            else:
                pass




        #////////////////////////////////////////////////////////////////////
        # 以下記述コード(各フェーズ内のtryに適切なものをコピペしてください)

        

        # "モーター"を使うコード
        # これらを使ってゴールの向きに回転し，ゴールへ前進する
        # モーターの動き部分の二行とtry，exceptをコピペ
        try:
            # モーターを回転して前進
            motor.accel(motor_right, motor_left)
            time.sleep(1)  # 何秒進むか

            # モーターの回転を停止
            motor.brake(motor_right, motor_left)
            time.sleep(1)  # 何秒進むか

            # モーターを回転させ，CanSatを1秒くらい右回転
            motor.rightturn(motor_right, motor_left)

            # モーターを回転させ，CanSatを1秒くらい左回転
            motor.leftturn(motor_right, motor_left)
            
            
            time.sleep(1)

        except Exception as e:
            print(f"An error occured in moving motor: {e}")
            # モーターを強制停止
            motor_left.value = 0.0
            motor_right.value = 0.0



        # "GPS"を使うコード
        # GPSを使う際は以下のコードを使用する
        # GPSデータを取得し，今いるlatitude, longtitudeを取得できる

        # UART(GPS)受信データ取得
        try:
            sentence = uart.readline()
            
        except Exception as e:
                print(f"An error occured in getting data from serial 0: {e}")


        # GPS緯度経度読み取り
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

        except Exception as e:
            print(f"An error occured in reading GPS tm, lat,lon: {e}")


        # "BNO055(9軸)"を使うコード
        # 地磁気Mag，ジャイロGyro，重力加速度を除く加速度Accel，除かない加速度Accel＿あｌｌ)を取得できる"
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
    


        # "BMP280(温度，気圧センサ)"を使うコード
        # 温度temperture，気圧pressure，相対高度altitudeを取得するコード
        # bmp280データ取得(基準高度参照)
        try:
            temperature = bmp.get_temperature()
            pressure = bmp.get_pressure()
            altitude = bmp.get_altitude(qnh=baseline)
            print(f"temperture{temperature:05.2f}*C")
            print(f"pressure: {pressure:05.2f}hPa")
            print(f"Relative altitude: {altitude:05.2f} metres")
        except Exception as e:
            print(f"An error occured in reading bmp: {e}")


        # "カメラ"を使うコード
        # カメラデータ取得&処理
        try:
            # フレームを取得
            if (CameraStart == False):
                picam2.start()
                CameraStart = True
            if (CameraStart == True):
                frame = picam2.capture_array()
                # 赤色を検出
                mask = cam.red_detect(frame)
                # 赤色検知の結果を取得
                # analize_redの戻り値は0が見つからない，1が中心，2が右，3が左，4がゴール，
                camera_order = cam.analyze_red(frame, mask)
                # 結果表示
                time.sleep(0.5)
                #print(len(contours))

        except Exception as e:
            print(f"An error occured in processing camera: {e}")

    # csvwrite.write()
    # csvの書き込みは準備中




# この.pyファイルがメイン関数の時のみ実行
if __name__ == "__main__":
    while True:
        main()
