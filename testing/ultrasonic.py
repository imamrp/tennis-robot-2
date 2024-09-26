import gpiozero
import RPi.GPIO as GPIO
import time

# setup
TRIG = 9
ECHO = 10

sensor = gpiozero.DistanceSensor(echo=ECHO,trigger=TRIG, max_distance=1.0)
print ("Waiting For Sensor To Settle")
time.sleep(1)

# measuring
for j in range(100):
    print('Distance: ', sensor.distance * 100)
    time.sleep(0.5)
