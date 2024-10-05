import time
import board
import adafruit_tcs34725
import RPi.GPIO as GPIO
import math

class DualSensorReader:
    SENSOR1_POWER_PIN = 17
    SENSOR2_POWER_PIN = 27
    LED_PIN = 4
    WAKEUP_TIME = 0.01

    def __init__(self, threshold = 50):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SENSOR1_POWER_PIN, GPIO.OUT)
        GPIO.setup(self.SENSOR2_POWER_PIN, GPIO.OUT)
        GPIO.setup(self.LED_PIN, GPIO.OUT)
        GPIO.output(self.SENSOR1_POWER_PIN, GPIO.LOW)
        GPIO.output(self.SENSOR2_POWER_PIN, GPIO.LOW)
        self.led_state = False
        self.base_colourR = None
        self.base_colourL = None
        self.threshold = threshold
        self.set_base_colour()

    def toggle_led(self):
        self.led_state = not self.led_state
        GPIO.output(self.LED_PIN, self.led_state)

    def init_sensor(self):
        i2c = board.I2C()
        sensor = adafruit_tcs34725.TCS34725(i2c)
        # gain 60 and 600 int time worked
        sensor.integration_time = 300
        sensor.gain = 1
        return sensor

    def read_sensor(self, sensor):
        r, g, b = sensor.color_rgb_bytes
        return r, g, b

    def read_both_sensors(self):
        GPIO.output(self.SENSOR1_POWER_PIN, GPIO.LOW)
        GPIO.output(self.SENSOR2_POWER_PIN, GPIO.HIGH)
        time.sleep(self.WAKEUP_TIME)
        sensorR = self.init_sensor()
        rgbR = self.read_sensor(sensorR)

        GPIO.output(self.SENSOR1_POWER_PIN, GPIO.HIGH)
        GPIO.output(self.SENSOR2_POWER_PIN, GPIO.LOW)
        time.sleep(self.WAKEUP_TIME)
        sensorL = self.init_sensor()
        rgbL = self.read_sensor(sensorL)

        return rgbR, rgbL
    
    def colour_difference(self, rgbA, rgbB):
        delta = (rgbA[0]-rgbB[0], rgbA[1]-rgbB[1], rgbA[2]-rgbB[2])
        result = math.sqrt(abs(delta[0])^2 + abs(delta[1])^2 + abs(delta[2])^2)
        return result

    def set_base_colour(self):
        self.base_colourR, self.base_colourL = self.read_both_sensors()

    def set_threshold(self, threshold):
        self.threshold = threshold
    
    def is_line_detected(self):
        rgbR, rgbL = self.read_both_sensors()
        print('left rgb', rgbL, 'right rgb', rgbR)
        detectR = self.colour_difference(rgbR, self.base_colourR) > self.threshold
        detectL = self.colour_difference(rgbL, self.base_colourL) > self.threshold
        print('left colour diff: ', self.colour_difference(rgbL, self.base_colourL), 'right colour diff: ', self.colour_difference(rgbR, self.base_colourR)) 
        return (detectL, detectR)

# Test
if __name__ == "__main__":
    reader = DualSensorReader(4)
    GPIO.output(4, GPIO.HIGH)
    while True:
        print(reader.is_line_detected())
        time.sleep(0.01)
