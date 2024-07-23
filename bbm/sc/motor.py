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