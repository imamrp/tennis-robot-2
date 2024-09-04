import time
import board
import adafruit_tcs34725
import RPi.GPIO as GPIO

i2c = board.I2C()
sensor = adafruit_tcs34725.TCS34725(i2c)

LED_PIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
led_state = False
GPIO.output(LED_PIN, led_state)

def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(LED_PIN, led_state)

def read_sensor():
    r, g, b = sensor.color_rgb_bytes
    return r, g, b

sensor.integration_time = 2.4
sensor.gain = 60

toggle_led()

while True:
    r, g, b = read_sensor()
    print('Color: ({0}, {1}, {2})'.format(r, g, b))

    time.sleep(0.5)
