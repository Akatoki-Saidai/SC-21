import time
from gpiozero import LED

# パラ分離用抵抗起動
input("NiCr go?")
start_time = time.time()

NiCr_PIN = LED(17)
NiCr_PIN.on()
print("NiCr wire turn on")

input("turn off?")

NiCr_PIN.off()
stop_time = time.time()
print("NiCr wire turn off. Parachute separated")
print("time: ", (stop_time - start_time), "_s")