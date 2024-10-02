import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
collect_motor = 22        
GPIO.setup(collect_motor, GPIO.OUT)
GPIO.output(collect_motor, GPIO.LOW)

for i in range(5):
    time.sleep(10)
    GPIO.output(collect_motor, GPIO.HIGH)

