import time
import board
import adafruit_tcs34725
import RPi.GPIO as GPIO

SENSOR1_POWER_PIN = 17
SENSOR2_POWER_PIN = 27

LED_PIN = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR1_POWER_PIN, GPIO.OUT)
GPIO.setup(SENSOR2_POWER_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)

GPIO.output(SENSOR1_POWER_PIN, GPIO.LOW)
GPIO.output(SENSOR2_POWER_PIN, GPIO.LOW)

def init_sensor():
    i2c = board.I2C()
    sensor = adafruit_tcs34725.TCS34725(i2c)
    sensor.integration_time = 2.4
    sensor.gain = 60
    return sensor

def read_sensor(sensor):
    r, g, b = sensor.color_rgb_bytes
    return r, g, b

led_state = False
def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(LED_PIN, led_state)

#toggle_led()

while True:
    # Turn on Sensor 1 and read data
    GPIO.output(SENSOR1_POWER_PIN, GPIO.LOW)
    GPIO.output(SENSOR2_POWER_PIN, GPIO.HIGH)
    time.sleep(0.005)
    sensor1 = init_sensor()
    r, g, b = read_sensor(sensor1)
    print('Sensor 1 Color: ({0}, {1}, {2})'.format(r, g, b))
    #time.sleep(0.1)

    # Turn on Sensor 2 and read data
    GPIO.output(SENSOR1_POWER_PIN, GPIO.HIGH)
    GPIO.output(SENSOR2_POWER_PIN, GPIO.LOW)
    time.sleep(0.005)
    sensor2 = init_sensor()
    r, g, b = read_sensor(sensor2)
    print('Sensor 2 Color: ({0}, {1}, {2})'.format(r, g, b))
    #time.sleep(0.1)
