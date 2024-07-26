
from sc import motor
import time


# モータードライバセットアップ
try:
	PIN_AIN1 = 23  # 17
	PIN_AIN2 = 4  # 4
	PIN_BIN1 = 5  # 16
	PIN_BIN2 = 13  # 13

	motor_right, motor_left = motor.setup(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2)

except Exception as e:
	print(f"An error occured in setting motor_driver: {e}")


motor_right.value = 0
motor_left.value = 0

while True:
	
	print("go?")
	input()
	try:
		# モーターを回転して前進
		motor.accel(motor_right, motor_left)
		print("motor: forward -1s")
		time.sleep(1)  # 何秒進むか

		# モーターの回転を停止
		motor.brake(motor_right, motor_left)
		print("motor: brake")
		time.sleep(1)  # 何秒進むか

	except Exception as e:
		print(f"An error occured in moving motor: {e}")
		# 停止
		motor_left.value = 0.0
		motor_right.value = 0.0


	time.sleep(1)

	try:
		# モーターを回転させ，CanSatを1秒くらい右回転
		motor.rightturn(motor_right, motor_left)
		print("motor: rightturn")

		time.sleep(1)

		# モーターを回転させ，CanSatを1秒くらい左回転
		motor.leftturn(motor_right, motor_left)
		print("motor: leftturn")        
		
		time.sleep(1)

	except Exception as e:
		print(f"An error occured in moving motor: {e}")
		# 停止
		motor_left.value = 0.0
		motor_right.value = 0.0