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


# mainゾーン

def main():
    
    # ---セットアップゾーン---
    
    # 少々ここらで(print関数を)オーバーライド
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

        #LEDもつけてみる
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
        baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])
    except Exception as e:
            print(f"An error occured in getting bmp280 data: {e}")

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

        # UART(GPS)受信データ取得
        try:
            sentence = uart.readline()
        except Exception as e:
                print(f"An error occured in getting data from serial 0: {e}")
                # print(len(sentence))
                # continue

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
            # 停止
            motor_left.value = 0.0
            motor_right.value = 0.0


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
            print(f"temperture{temperature:05.2f}*C")
            print(f"pressure: {pressure:05.2f}hPa")
            print(f"Relative altitude: {altitude:05.2f} metres")
        except Exception as e:
            print(f"An error occured in reading bmp280: {e}")


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
                # 面積のもっとも大きい領域を表示
                camera_order = cam.analyze_red(frame, mask)
                # 結果表示
                time.sleep(0.5)
                #print(len(contours))

        except Exception as e:
            print(f"An error occured in processing camera: {e}")




# この.pyファイルがメイン関数の時のみ実行
if __name__ == "__main__":
    while True:
        main()
