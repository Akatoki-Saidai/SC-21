# BMPとBNO(とGPS)を受信してメモリリーク"する"プログラム


import gc
import psutil
import time
from smbus2 import SMBus
import serial
from gpiozero import LED

# scに使用ライブラリほぼまとめました
import sc.print_override as override
from sc.bmp280 import BMP280
from sc.bno055 import BNO055
from sc.micropyGPS import MicropyGPS
import sc.csv_print as csv


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
        csv.print('serious_error', f"An error occured in setting serial 0: {e}")


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
        csv.print('error', f"An error occured in turn on bmp, bno, led: {e}")


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
        bmp280 = BMP280(i2c_dev=bus)

        temperature = []
        pressure = []
    except Exception as e:
        print(f"An error occured in setting bmp280 object: {e}")
        csv.print('serious_error', f"An error occured in setting bmp280 object: {e}")

    # bmp280高度算出用基準気圧取得
    try:
        baseline_values = []
        baseline_size = 20

        for i in range(baseline_size):
            pressure = bmp280.get_pressure()
            baseline_values.append(pressure)
            time.sleep(0.5)
        baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])
        csv.print('alt_base_press', baseline)
    except Exception as e:
        print(f"An error occured in getting bmp280 data: {e}")
        csv.print('serious_error', f"An error occured in getting bmp280 data: {e}")


    # ---繰り返しゾーン---

    while True:

        # UART(GPS)受信データ取得
        try:
            sentence = uart.readline()
        except Exception as e:
            print(f"An error occured in getting data from serial 0: {e}")
            csv.print('error', f"An error occured in getting data from serial 0: {e}")
            # print(len(sentence))
            # continue

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
                                csv.print('error', f"An error occured in updating GPS data: {e}")
                        
                        if stat:
                            try:
                                tm = gnss.timestamp
                                # tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                                latitude, longitude = gnss.latitude[0], gnss.longitude[0]
                                # print('=' * 20)
                                print(gnss.date_string(), tm[0], tm[1], int(tm[2]))
                                print("latitude:", gnss.latitude[0])
                                print("longitude:", gnss.longitude[0])
                                csv.print('lat', gnss.latitude)
                                csv.print('lon', gnss.longitude)
                            except Exception as e:
                                print(f"An error occured in loading GPS data : {e}")
                                csv.print('error', f"An error occured in loading GPS data : {e}")

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
            csv.print('error', f"An error occured in reading bno055: {e}")
    
        # bmp280データ取得(基準高度参照)
        try:
            temperature = bmp280.get_temperature()
            pressure = bmp280.get_pressure()
            altitude = bmp280.get_altitude(qnh=baseline)
            print(f"temperture{temperature:05.2f}*C")
            print(f"pressure: {pressure:05.2f}hPa")
            print(f"Relative altitude: {altitude:05.2f} metres")
            
            
            
            # ここ決してマネしないでください
            temperature.append(pressure)
            pressure.append(temperature)

        except Exception as e:
            print(f"An error occured in reading bmp280: {e}")
            csv.print('error', f"An error occured in reading bmp280: {e}")


        try:
            
            for i in range(100):
                memory_state = psutil.virtual_memory()
                print("memory: ", memory_state.percent)
                csv.print('msg', f'memory : {memory_state.percent}')
                time.sleep(0.1)

            gc.collect()
            memory_state = psutil.virtual_memory()
            print("memory: ", memory_state.percent)
            csv.print('msg', f'memory : {memory_state.percent}')

        except Exception as e:
             print(f"An error occured in surveying memoryleak: {e}")
             csv.print('serious_error', f"An error occured in surveying memoryleak: {e}")



# この.pyファイルがメイン関数の時のみ実行
if __name__ == "__main__":
    while True:
        main()
