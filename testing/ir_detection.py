import RPi.GPIO as GPIO
import time

IR_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

initial_state = GPIO.input(IR_PIN)
while 1:
    if GPIO.input(IR_PIN) != initial_state:
        print("line detected")
    time.sleep(0.2)