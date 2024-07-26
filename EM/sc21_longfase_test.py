import numpy as np
import math

#----------------------------------------------------------
#long faseで使う関数を定義しています。この中身はcalc_xy.pyの中にいるのでEMにコピーする必要はありません
def calc_xy(phi_deg, lambda_deg, phi0_deg, lambda0_deg):
    """ 緯度経度を平面直角座標に変換する
    - input:
        (phi_deg, lambda_deg): 変換したい緯度・経度[度]（分・秒でなく小数であることに注意）
        (phi0_deg, lambda0_deg): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
    - output:
        x: 変換後の平面直角座標[m]
        y: 変換後の平面直角座標[m]
    """
    # 緯度経度・平面直角座標系原点をラジアンに直す
    phi_rad = np.deg2rad(phi_deg)
    lambda_rad = np.deg2rad(lambda_deg)
    phi0_rad = np.deg2rad(phi0_deg)
    lambda0_rad = np.deg2rad(lambda0_deg)

    # 補助関数
    def A_array(n):
        A0 = 1 + (n**2)/4. + (n**4)/64.
        A1 = -     (3./2)*( n - (n**3)/8. - (n**5)/64. ) 
        A2 =     (15./16)*( n**2 - (n**4)/4. )
        A3 = -   (35./48)*( n**3 - (5./16)*(n**5) )
        A4 =   (315./512)*( n**4 )
        A5 = -(693./1280)*( n**5 )
        return np.array([A0, A1, A2, A3, A4, A5])

    def alpha_array(n):
        a0 = np.nan # dummy
        a1 = (1./2)*n - (2./3)*(n**2) + (5./16)*(n**3) + (41./180)*(n**4) - (127./288)*(n**5)
        a2 = (13./48)*(n**2) - (3./5)*(n**3) + (557./1440)*(n**4) + (281./630)*(n**5)
        a3 = (61./240)*(n**3) - (103./140)*(n**4) + (15061./26880)*(n**5)
        a4 = (49561./161280)*(n**4) - (179./168)*(n**5)
        a5 = (34729./80640)*(n**5)
        return np.array([a0, a1, a2, a3, a4, a5])

    # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
    m0 = 0.9999 
    a = 6378137.
    F = 298.257222101

    # (1) n, A_i, alpha_iの計算
    n = 1. / (2*F - 1)
    A_array = A_array(n)
    alpha_array = alpha_array(n)

    # (2), S, Aの計算
    A_ = ( (m0*a)/(1.+n) )*A_array[0] # [m]
    S_ = ( (m0*a)/(1.+n) )*( A_array[0]*phi0_rad + np.dot(A_array[1:], np.sin(2*phi0_rad*np.arange(1,6))) ) # [m]

    # (3) lambda_c, lambda_sの計算
    lambda_c = np.cos(lambda_rad - lambda0_rad)
    lambda_s = np.sin(lambda_rad - lambda0_rad)

    # (4) t, t_の計算
    t = np.sinh( np.arctanh(np.sin(phi_rad)) - ((2*np.sqrt(n)) / (1+n))*np.arctanh(((2*np.sqrt(n)) / (1+n)) * np.sin(phi_rad)) )
    t_ = np.sqrt(1 + t*t)

    # (5) xi', eta'の計算
    xi2  = np.arctan(t / lambda_c) # [rad]
    eta2 = np.arctanh(lambda_s / t_)

    # (6) x, yの計算
    x = A_ * (xi2 + np.sum(np.multiply(alpha_array[1:],
                                       np.multiply(np.sin(2*xi2*np.arange(1,6)),
                                                   np.cosh(2*eta2*np.arange(1,6)))))) - S_ # [m]
    y = A_ * (eta2 + np.sum(np.multiply(alpha_array[1:],
                                        np.multiply(np.cos(2*xi2*np.arange(1,6)),
                                                    np.sinh(2*eta2*np.arange(1,6)))))) # [m]
    # return
    return x, y # [m]

def Rotation_clockwise_xy(vec_xy,radian):
    sin_rad = np.sin(radian)
    cos_rad = np.cos(radian)
    new_vector_x = vec_xy[0]*cos_rad + vec_xy[1]*sin_rad
    new_vector_y = vec_xy[1]*cos_rad - vec_xy[0]*sin_rad
    new_vector = (new_vector_x,new_vector_y)

    return new_vector
#----------------------------------------------------------


#--------------------Test code-----------------------------
#Test用に緯度経度を手動で入れれるようにしています。(EMにはいらないコード)
goal = (35.86513476045121, 139.6074040979085)
goal_latitude = goal[0]
goal_longtitude = goal[1]
cansat = (35.86505976922859, 139.60741482674408)
latitude = cansat[0]
longtitude = cansat[1]
Mag = (0.000000,1.000000)
#----------------------------------------------------------
#ここから下をEMにコピーしてください。Test用にモーターを動かす部分とデータ取得の部分をコメントアウトしているので動かす際には復活させといてください

#以下がデータ取得部分。コピペして、コメントアウトを外す際にはインデントに注意してください。
# # UART(GPS)受信データ取得
# try:
# sentence = uart.readline()

# except Exception as e:
# print(f"An error occured in getting data from serial 0: {e}")

# # GPS緯度経度読み取り
# try:
# if len(sentence) > 0:
# for x in sentence:
#     if 10 <= x <= 126:
#         try:
#             stat = gnss.update(chr(x))
#         #print("stat:",stat,"x:",x,"chr:",chr(x))
#         #print(chr(x))
#         except Exception as e:
#                 print(f"An error occured in updating GPS data: {e}")
        
#         if stat:
#             try:
#                 tm = gnss.timestamp
#                 # tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
#                 latitude, longtitude = gnss.latitude[0], gnss.longitude[0]
#                 # print('=' * 20)
#                 print(gnss.date_string(), tm[0], tm[1], int(tm[2]))
#                 print("latitude:", gnss.latitude[0])
#                 print("longitude:", gnss.longitude[0])
#             except Exception as e:
#                     print(f"An error occured in loading GPS data : {e}")

# except Exception as e:
# print(f"An error occured in reading GPS tm, lat,lon: {e}")


# # "BNO055(9軸)"を使うコード
# # 地磁気Mag，ジャイロGyro，重力加速度を除く加速度Accel，除かない加速度Accel＿あｌｌ)を取得できる"
# # bno055データ取得
# try:
# #Gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)
# Mag = bno.getVector(BNO055.VECTOR_MAGNETOMETER)
# #Accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
# #Accel_all = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
# #print("Gyro: ", Gyro)
# print("Mag: ", Mag)
# #print("Accel", Accel)
# #print("Accel_all", Accel_all)
# except Exception as e:
# print(f"An error occured in reading bno055: {e}")

#以下は取得したデータを処理するコードです。
#1.ゴールの緯度経度をCanSat中心のxy座標で表す。
goal_xy = calc_xy(goal_latitude,goal_longtitude,latitude,longtitude)
#2.緯度経度→→→ゴールと機体の距離を求める
print(goal_xy)
print(goal_xy[0])
cansat_to_goal_y_sq = (goal_xy[1])**2
cansat_to_goal_x_sq = (goal_xy[0])**2
distance = np.sqrt(cansat_to_goal_x_sq + cansat_to_goal_y_sq)
#3.機体の正面と北の向きの関係＋北の向きとゴールの向きの関係→→→機体の正面とゴールの向きの関係を求める
#やってることとしては東西南北の基底→CanSatの基底に座標変換するために回転行列を使ってる感じ
#North_angle_rad - math.piは、平面直交座標のx軸(西)と北の向きを表すときのx軸(機体の正面)が何度ずれているかを表している
North_angle_rad = np.arctan2(Mag[1],Mag[0])
cansat_to_goal = Rotation_clockwise_xy(goal_xy,North_angle_rad - math.pi)
#4.CanSatの正面とゴールの向きの関係を角度で表現している(radian→degreeの変換も行う)。ただし、角度の定義域は(0<=degree<=360)。正面は0と360で真後ろが180。
cansat_to_goal_angle = np.arctan2(cansat_to_goal[1],cansat_to_goal[0])
cansat_to_goal_angle_degree = math.degrees(cansat_to_goal_angle) + 180
#5.機体の正面とゴールの向きの関係から、右に曲がるか、左に曲がるか、正面に進むか判断する
print(cansat_to_goal_angle_degree)
print(distance)
if (cansat_to_goal_angle_degree < 30) or (330 < cansat_to_goal_angle_degree):
    print("forward\n")
    # motor.accel(motor_right, motor_left)
    # time.sleep(3)
if (30 < cansat_to_goal_angle_degree <=135):
    print("right\n")
    # motor.rightturn(motor_right, motor_left)
    # time.sleep(1)
if (135 <= cansat_to_goal_angle_degree <= 180):
    print("sharp_right\n")
    # motor.rightturn(motor_right, motor_left)
    # time.sleep(2)]
if (225 < cansat_to_goal_angle_degree <= 330):
    print("left\n")
    # motor.leftturn(motor_right, motor_left)
    # time.sleep(1) 
if (180 < cansat_to_goal_angle_degree < 225):
    print("sharp_left\n")
    # motor.leftturn(motor_right, motor_left)
    # time.sleep(2)  