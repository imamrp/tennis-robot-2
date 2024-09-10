import time
import busio
import adafruit_tcs34725
import RPi.GPIO as GPIO

i2c1 = busio.I2C(scl=board.D7, sda=board.D8)
i2c2 = busio.I2C(scl=board.D11, sda=board.D10)

sensor1 = adafruit_tcs34725.TCS34725(i2c1)
sensor2 = adafruit_tcs34725.TCS34725(i2c2)

LED_PIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
led_state = False
GPIO.output(LED_PIN, led_state)

def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(LED_PIN, led_state)

def read_sensor(sensor):
    r, g, b = sensor.color_rgb_bytes
    return r, g, b

sensor1.integration_time = 2.4
sensor1.gain = 60
sensor2.integration_time = 2.4
sensor2.gain = 60

toggle_led()

while True:
    r1, g1, b1 = read_sensor(sensor1)
    print('Sensor 1 Color: ({0}, {1}, {2})'.format(r1, g1, b1))

    r2, g2, b2 = read_sensor(sensor2)
    print('Sensor 2 Color: ({0}, {1}, {2})'.format(r2, g2, b2))

    time.sleep(0.5)
