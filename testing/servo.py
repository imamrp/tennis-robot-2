from gpiozero import Servo, PWMOutputDevice
from time import sleep

# Connect servo to GPIO11
PWM_pin = 11
servo = Servo(PWM_pin,min_pulse_width=0.001, max_pulse_width=0.002,frame_width=0.0025)    # min and max pulse width may need to be changed if rom not large enough

def ctrl_gate(servo, open):
    servo.value = 1 if open else -1 # Open gate
    sleep(1)

# for j in range(2):
#     print("close")
#     ctrl_gate(servo,open=False)
#     sleep(3)
#     print('Open')
#     ctrl_gate(servo,open=True)
#     sleep(1)
#     print(j)

# send servo to pos 0
print("open")
ctrl_gate(servo,open=True)
sleep(3)
print("close")
ctrl_gate(servo,open=False)

