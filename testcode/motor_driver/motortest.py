from gpiozero import PWMLED
import time

a = PWMLED(13)
b = PWMLED(16)
c = PWMLED(17)
d = PWMLED(4)

a.value = 0
b.value = 0
c.value = 0
d.value = 0

print("go?")
input()

while True:
	a.value = 0
	b.value = 1
	c.value = 1
	d.value = 0
	time.sleep(1)
