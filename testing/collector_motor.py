import RPi.GPIO as GPIO
import time

collect_motor = 22        
GPIO.setup(collect_motor, GPIO.OUT)
GPIO.output(collect_motor, GPIO.LOW)

for i in range(5):
    time.sleep(3)
    GPIO.output(collect_motor, GPIO.HIGH)
    time.sleep(3)        # TODO: confirm if good amount of sleep time
    GPIO.output(collect_motor, GPIO.LOW)
