from gpiozero import Servo, PWMOutputDevice
import RPi.GPIO as GPIO
import time

# Connect servo to GPIO11
PWM_pin = 11
# setup the GPIO pin for the servo
servo_pin = 11
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)

# setup PWM process
pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)

pwm.start(7) # start PWM by rotating to 90 degrees

for i in range(5):
    pwm.ChangeDutyCycle(5.0) # rotate to 0 degrees
    time.sleep(3)
    pwm.ChangeDutyCycle(12.0) # rotate to 180 degrees
    time.sleep(3)
    pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(3)

pwm.ChangeDutyCycle(0) # this prevents jitter
pwm.stop() # stops the pwm on 13
GPIO.cleanup() # good practice when finished using a pin
# servo = Servo(PWM_pin,min_pulse_width=0.005, max_pulse_width=0.0176,frame_width=0.02)    # min and max pulse width may need to be changed if rom not large enough

# def ctrl_gate(servo, open):
#     servo.value = -1 if open else 1 # Open gate
#     time.sleep(1)

# for j in range(5):
#     print("close")
#     ctrl_gate(servo,open=False)
#     time.sleep(3)
#     print('Open')
#     ctrl_gate(servo,open=True)
#     time.sleep(1)
#     print(j)

# # send servo to pos 0
# ctrl_gate(servo, False)
# while True:
#     pass

