import time
from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory



def setup(AIN1, AIN2, BIN1, BIN2):

    dcm_pins = {
                "left_forward": AIN2,
                "left_backward": AIN1,
                "right_forward": BIN2,
                "right_backward": BIN1,
            }

    factory = PiGPIOFactory()
    left = Motor( forward=dcm_pins["left_forward"],
                        backward=dcm_pins["left_backward"],
                        pin_factory=factory)
    right = Motor( forward=dcm_pins["right_forward"],
                        backward=dcm_pins["right_backward"],
                        pin_factory=factory)
    
    return right, left


def accel(right, left):
    power = 0
    for i in range(int(1 / 0.05)):
        right.value = power
        left.value = power
        power += 0.05

    right.value = 1
    left.value = 1


def brake(right, left):
    power_r = right.value
    power_l = left.value

    for i in range(int(1 / 0.05)):
        right.value = power_r
        left.value = power_l
        if power_r > 0:
            power_r -= 0.05
        elif power_r < 0:
            power_r += 0.05
        else:
            pass
        if power_l > 0:
            power_l -= 0.05
        elif power_l < 0:
            power_l += 0.05
        else:
            pass

    

    right.value = 0
    left.value = 0


def rightturn(right, left):
    
    right.value = 0
    left.value = 0
    power = 0
    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            right.value = power
            left.value = -1 * power
        
        power += 0.05

    power = 1
    right.value = 1
    left.value = -1

    time.sleep(1)

    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            right.value = power
            left.value = -1 * power
        
        power -= 0.05

    right.value = 0
    left.value = 0




def leftturn(right, left):
    
    right.value = 0
    left.value = 0
    power = 0
    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            right.value = -1 * power
            left.value = power
        
        power += 0.05

    power = 1
    right.value = -1
    left.value = 1

    time.sleep(1)

    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            right.value = -1 * power
            left.value = power
        
        power -= 0.05
            
    right.value = 0
    left.value = 0

    



def rightonly(right, left):
    
    right.value = 0
    left.value = 0

    power = 0
    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            right.value = power

        power += 0.05

    power = 1
    right.value = 1

    time.sleep(1)

    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            right.value = power
            
        power -= 0.05

    right.value = 0




def leftonly(right, left):
    
    right.value = 0
    left.value = 0
    power = 0

    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            left.value = power
        
        power += 0.05

    power = 1
    left.value = 1

    time.sleep(1)

    for i in range(int(1 / 0.05)):
        if (-1 <= power <= 1):
            left.value = power
        
        power -= 0.05
        
    left.value = 0