from gpiozero import PWMLED
from time import sleep

gp_s = {}

while True:
	gp = int(input('\nGPIO:'))
	if gp not in gp_s:
		gp_s[gp] = PWMLED(gp)
	gp_s[gp].value = float(input('Value:'))
	sleep(0.2)
