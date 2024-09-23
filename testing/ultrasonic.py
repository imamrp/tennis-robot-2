import gpiozero
import RPi.GPIO as GPIO
import time

# setup
GPIO.setmode(GPIO.BCM)

TRIG = 3
ECHO = 2

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

GPIO.output(TRIG, False)
sensor = gpiozero.DistanceSensor(echo=ECHO,trigger=TRIG)
print ("Waiting For Sensor To Settle")
time.sleep(1)

# measuring
for j in range(100):
    print('Distance: ', sensor.distance * 100)
    time.sleep(0.5)

GPIO.cleanup() 
