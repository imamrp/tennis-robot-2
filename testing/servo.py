from gpiozero import Servo, PWMOutputDevice
from time import sleep

# Connect servo to GPIO11
PWM_pin = 11
servo = Servo(PWM_pin,min_pulse_width=0.001, max_pulse_width=0.002,frame_width=0.0025)    # min and max pulse width may need to be changed if rom not large enough

for j in range(10):
    servo.value = 1
    print('Close')
    sleep(3)
    servo.value = -1
    print('Open')
    sleep(1)

# send servo to pos 0
servo.value = 1
