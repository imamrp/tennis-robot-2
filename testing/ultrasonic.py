import gpiozero
import RPi.GPIO as GPIO
import time

# setup
TRIG = 27
ECHO = 17

sensor = gpiozero.DistanceSensor(echo=ECHO,trigger=TRIG)
print ("Waiting For Sensor To Settle")
time.sleep(1)

# measuring
for j in range(100):
    print('Distance: ', sensor.distance * 100)
    time.sleep(0.5)

GPIO.cleanup() 
