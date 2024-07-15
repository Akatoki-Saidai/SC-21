import bmp280 as BMP280
import bno055 as BNO055
import gnss

def main():
    
    #gnssのセットアップ
    # シリアル通信設定
    uart = serial.Serial('/dev/serial0', 38400, timeout = 10)
    # gps設定
    my_gps = MicropyGPS(9, 'dd')
    # 10秒ごとに表示
    tm_last = 0

    #bno055のセットアップ
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setExternalCrystalUse(True)

    #bmp280のセットアップ
    bus = SMBus(1)
    bmp280 = BMP280(i2c_dev=bus)
    #-----------
    # baseline_values = []
    # baseline_size = 5

    # for i in range(baseline_size):
    #     pressure = bmp280.get_pressure()
    #     baseline_values.append(pressure)
    #     time.sleep(0.1)

    # baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])


    while True:
        sentence = uart.readline()
        #print(len(sentence))
        #continue
        if len(sentence) > 0:
            #print("4")
            for x in sentence:
                if 10 <= x <= 126:
                    #print("5")
                    stat = my_gps.update(chr(x))
                    #print("stat:",stat,"x:",x,"chr:",chr(x))
                    #print(chr(x))
                    if stat:
                        #print("6")
                        tm = my_gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            print('=' * 20)
                            print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
	# 変数定義、初期化
        print("Gyro",bno.getVector(BNO055.VECTOR_GYROSCOPE))
        print("Mag",bno.getVector(BNO055.VECTOR_MAGNETOMETER))
        print("Accel",bno.getVector(BNO055.VECTOR_LINEARACCEL))
        print("Accel_all",bno.getVector(BNO055.VECTOR_ACCELEROMETER))
        temperature = bmp280.get_temperature()
        pressure = bmp280.get_pressure()
        print(f"{temperature:05.2f}*C {pressure:05.2f}hPa")
        # altitude = bmp280.get_altitude(qnh=baseline)
        # print(f"Relative altitude: {altitude:05.2f} metres")
        time.sleep(1)

    






if __name__ == "__main__":
    main()