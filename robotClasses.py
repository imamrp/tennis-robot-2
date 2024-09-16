""" 
By: Imam Prakoso
This file contains the DiffDriveRobot class which is what controls the robot's movement with its two motors. 
It is recomended that the robot's motors be continuously controlled in a separate process while the main process
sets the target angular and positional velocity of the robot.  
"""

import time, math, random, multiprocessing
import matplotlib.pyplot as plt
import numpy as np
import RPi.GPIO as GPIO
import gpiozero

class DiffDriveRobot:
    """DiffDriveRobot class initiates the robot and its drive control. This class is to be called in a rapidly repeating process aside from the main process so that the motor can be continuously controlled. 
    """
    def __init__(self, dt=0.075, Kp=0.2, Ki=0, Kd=0.03, wheel_radius=0.027, wheel_sep=0.22):
        self.x = 0.0 # y-position (m)
        self.y = 0.0 # y-position (m)
        self.th = 0.0 # orientation (rad)
        
        self.wL = 0.0 #rotational velocity left wheel (rad/s)
        self.wR = 0.0 #rotational velocity right wheel
        
        self.dt = dt # Pause time for calculating measured speed
        
        self.r = wheel_radius
        self.l = wheel_sep

        # Set pins for the RIGHT motor driver
        self.motor_A_in1 = 19
        self.motor_A_in2 = 26
        self.motor_A_en = 13 # right motor

        # Set pins for the LEFT motor driver
        self.motor_B_in2 = 20
        self.motor_B_in1 = 21
        self.motor_B_en = 12 # left motor

        # Set GPIO modes
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor_A_in1, GPIO.OUT)
        GPIO.setup(self.motor_A_in2, GPIO.OUT)
        GPIO.setup(self.motor_B_in1, GPIO.OUT)
        GPIO.setup(self.motor_B_in2, GPIO.OUT)
        GPIO.setup(self.motor_A_en, GPIO.OUT)
        GPIO.setup(self.motor_B_en, GPIO.OUT)

        # Create PWM instance with a frequency of 500 Hz
        self.pwm_L = GPIO.PWM(self.motor_B_en, 500)
        self.pwm_R = GPIO.PWM(self.motor_A_en, 500)

        # Start PWM with a duty cycle of 0%
        self.pwm_L.start(0)
        self.pwm_R.start(0)

        # setting encoders
        self.encoderR = gpiozero.RotaryEncoder(a=5, b=6,max_steps=100000)
        self.encoderL = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)
        
        # Controller parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_prev_L = 0 # Previous error
        self.error_prev_R = 0
        self.error_sum_L = 0 # Error accumulation
        self.error_sum_R = 0
        self.duty_cycle_L = 0 
        self.duty_cycle_R = 0
        
    # Motor speed and sending power to it
    def get_motor_angular_velocity(self):
        """Gets the angular velocity of motor from the encoder

        Returns:
            Tuple(Float,Float): The left and right angular veloctiy in rad/s respectively
        """
        # Setting encoder reference
        encoderL_start = self.encoderL.steps 
        encoderR_start = self.encoderR.steps 

        time.sleep(self.dt)

        # delta encoder values
        delta_encoderL = self.encoderL.steps - encoderL_start
        delta_encoderR = self.encoderR.steps - encoderR_start
        
        # Counts per second
        countsPerSecondR = -self.encoderR.steps/self.dt
        countsPerSecondL = -self.encoderL.steps/self.dt

        # Find angular velocity (Rough speed limit is 1400 counts per second) TODO: test limit on different surfaces
        # 900 counts in 1 revolution
        wR = (2*math.pi) * countsPerSecondR / 900
        wL = (2*math.pi) * countsPerSecondL / 900

        return wL, wR # in radians per second limit approx. 9.8 rad/s

    def rotate_motor(self, dutyCycle:int, motor:str):
        """Rotates a specified motor by a set duty cycle

        Args:
            dutyCycle (int): Int from 0-100 showing the duty cycle
            motor (str): 'l' specifies the left motor whereas 'r' specifies the right motor

        Raises:
            Exception: If 'l' or 'r' not specified then the motor control will be invalid.
        """
        if motor == 'r':        # right motor (assuming motor A from code)
            if dutyCycle >= 0:  # forward
                GPIO.output(self.motor_A_in1, GPIO.HIGH)
                GPIO.output(self.motor_A_in2, GPIO.LOW)
            else:               # backward
                GPIO.output(self.motor_A_in1, GPIO.LOW)
                GPIO.output(self.motor_A_in2, GPIO.HIGH)
            self.pwm_R.ChangeDutyCycle(abs(dutyCycle))
            
        elif motor == 'l':      # left motor (assuming motor B from code)
            if dutyCycle >= 0:  # forward
                GPIO.output(self.motor_B_in1, GPIO.HIGH)
                GPIO.output(self.motor_B_in2, GPIO.LOW)
            else:               # backward
                GPIO.output(self.motor_B_in1, GPIO.LOW)
                GPIO.output(self.motor_B_in2, GPIO.HIGH)
            self.pwm_L.ChangeDutyCycle(abs(dutyCycle))
            
        else:
            raise Exception("Incorrect motor specified. Please specify either 'l' for left or 'r' for right motor")
        
        return

    # Velcoity and position updates
    def get_base_velocity(self, wL:float, wR:float):
        """Calculates the positional and rotational veloctiy of the robot based on the left and right motor's angular velocity

        Args:
            wl (float): Angular velocity of left motor in rad/s
            wr (float): Angular velocity of right motor in rad/s

        Returns:
            Tuple(Float,Float): The velocity v (m/s) and angular velocity w (rad/s) of the robot. +v is forwards and +w is clockwise turning.
        """
        v = (wL*self.r + wR*self.r)/2.0
        
        w = (wL*self.r - wR*self.r)/self.l
        
        return v, w

    def position_update(self):
        """Updates the robot's position (x,y) and its angle (th) relative to the starting point

        Returns:
            Tuple(float,float,float): x,y,th
        """       
        # v, w = self.get_base_velocity(self.wL,self.wR)

        # updating movement estimating it as a turn then straight
        encoder_difference = self.encoderL.steps - self.encoderR.steps

        straight_movements = min(-self.encoderR.steps, -self.encoderL.steps) + encoder_difference / 2

        # rotation
        rotation_wheel_dist = (encoder_difference / 900) * self.r * np.pi        # distance each of the wheels go around the centre of rotaion
        turning_circle = np.pi * self.l
        delta_th = (rotation_wheel_dist / turning_circle) * 2*np.pi

        # first straight movement
        # getting distance travelled
        dist_1 = 2 * self.r * np.pi * straight_movements / 900 

        # updating theta first
        self.th = self.th + delta_th

        # updating x and y with updated theta
        self.x = self.x + dist_1 * np.cos(self.th)
        self.y = self.y + dist_1 * np.sin(self.th)

        self.encoderL.steps = 0
        self.encoderR.steps = 0
        
        return self.x, self.y, self.th

    # Motor control and driving
    # TODO: implement with multiprocess
    def motor_controller(self, wL_desired:float, wL_measured:float, wR_desired:float, wR_measured:float, error_prev_L:float, error_prev_R:float, error_sum_L:float, error_sum_R:float, prev_cycle_L:int, prev_cycle_R:int):
        """Controller for both left and right motors implementing a PI controller

        Args:
            wL_desired (float): Desired angular velocity (rad/s) of the left motor
            wL_measured (float): Measured angular velocity (rad/s) of the left motor
            wR_desired (float): Desired angular velocity (rad/s) of the right motor
            wR_measured (float): Measured angular velocity (rad/s) of the right motor
            error_prev_L (float): Previous error in left motor
            error_prev_R (float): Previous error in right motor
            error_sum_L (float): Accumulative sum of the error in left motor
            error_sum_R (float): Accumulative sum of the error in right motor
            prev_cycle_L (int): The previous duty cycle of the left motor from 0-100
            prev_cycle_R (int): The previous duty cycle of the right motor from 0-100

        Returns:
            Tuple(int,int,float,float): Duty cycle of the left motor, duty cycle of the right motor, New accumulative sum of the error in left motor, and new accumulative sum of the error in right motor
        """
        # PI Controller: finding change in duty cycle needed to get to reference speed 
        if (wL_desired == 0 and wR_desired == 0): # for stopping
            duty_cycle_L = 0
            duty_cycle_R = 0
        else:
            delta_duty_cycle_L = self.Kp*(wL_desired-wL_measured) + self.Ki*error_sum_L*self.dt + self.Kd*error_prev_L/self.dt
            delta_duty_cycle_R = self.Kp*(wR_desired-wR_measured) + self.Ki*error_sum_R*self.dt + self.Kd*error_prev_R/self.dt
        
            # Getting new duty cycle
            duty_cycle_L = min(max(-100,prev_cycle_L + delta_duty_cycle_L),100)
            duty_cycle_R = min(max(-100,prev_cycle_R + delta_duty_cycle_R),100)

        # Error previous
        error_prev_L = wL_desired-wL_measured
        error_prev_R = wR_desired-wR_measured
        
        # Error accumulation
        error_sum_L = error_sum_L + (wL_desired-wL_measured)
        error_sum_R = error_sum_R + (wR_desired-wR_measured)
        
        return duty_cycle_L, duty_cycle_R, error_prev_L, error_prev_R, error_sum_L, error_sum_R
    
    def drive(self,v_desired:float,w_desired:float):
        """Drives the robot for a desired velcotiy v (m/s) and desired rotational velocity w (rad/s) by using a PI controller and the measured velocity.

        Args:
            v_desired (float): The desired velocity (m/s) of the robot. Positive means driving forward
            w_desired (float): The desired rotational velocity (rad/s) of the robot. Positive means counter clockwise turning (left turning).

        Returns:
            Tuple(float,float,float,float): The left and right PWM duty cycles, left motor desired and measured speed, right motor desired and measured speed. 
        """
        # Calculate desired left and right motor speed
        wL_desired = (v_desired - self.l*w_desired/2)/self.r
        wR_desired = (v_desired + self.l*w_desired/2)/self.r

        # Measure the current speed of motors
        wL_measured, wR_measured = self.get_motor_angular_velocity()
        self.wL = (wL_measured + self.wL)/2 # averaging out
        self.wR = (wR_measured + self.wR)/2 # averaging out
        
        # Use PID controller
        self.duty_cycle_L, self.duty_cycle_R, self.error_prev_L, self.error_prev_R, self.error_sum_L, self.error_sum_R = self.motor_controller(wL_desired = wL_desired, 
                                                                                                                                                wL_measured = self.wL,
                                                                                                                                                wR_desired = wR_desired, 
                                                                                                                                                wR_measured = self.wR, 
                                                                                                                                                error_prev_L = self.error_prev_L,
                                                                                                                                                error_prev_R = self.error_prev_R,
                                                                                                                                                error_sum_L = self.error_sum_L, 
                                                                                                                                                error_sum_R = self.error_sum_R, 
                                                                                                                                                prev_cycle_L= self.duty_cycle_L, 
                                                                                                                                                prev_cycle_R= self.duty_cycle_R)
        
        print(f"x: {self.x}, y: {self.y}, th: {self.th}")
        # Send PWM to wheels
        self.rotate_motor(dutyCycle=self.duty_cycle_L, motor="l")
        self.rotate_motor(dutyCycle=self.duty_cycle_R, motor="r")
        
        # Position update
        self.position_update()
        
        return self.duty_cycle_L, self.duty_cycle_R, wL_desired, self.wL, wR_desired, self.wR

### Tests for the code
# Multiprocessing code
def set_speed_process(v_desired,w_desired):
    """Process that randomly sets speed of the robot. This can be thought of the 'main' process of the robot. 

    Args:
        v_desired (float): The desired speed of the robot
    """
    while True:
        # if bool(random.getrandbits(1)):
        v_desired.value = float(random.choice([-0.1,0,0.1]))
        w_desired.value = float(0)
        print(f"\n\n\n\nNew Speed: v={v_desired.value}; w={w_desired.value}")
        time.sleep(5)
        # else:
        #     w_desired.value = float(random.choice([-0.5,0,0.5]))
        #     v_desired.value = float(0)
        #     print(f"\n\n\n\nNew Speed: v={v_desired.value}; w={w_desired.value}")
        #     time.sleep(5)
        
def robot_control_process(v_desired,w_desired):
    """The process that controls the robot's motors continuously and repeatedly.

    Args:
        v_desired (float): The desired speed of the robot
    """
    robot = DiffDriveRobot()
    
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=v_desired.value, w_desired=w_desired.value)
        print(f"duty_cycle_L: {duty_cycle_L:5.2f}, duty_cycle_R: {duty_cycle_R:5.2f}, wL_desired: {wL_desired:5.2f}, wL_measured: {wL_measured:5.2f}, wR_desired: {wR_desired:5.2f}, wR_measured: {wR_measured:5.2f}")
        
def test_multiprocess():
    """Test function to see the functionality of the robot class.
    """
    v_desired = multiprocessing.Value('f', 0)
    w_desired = multiprocessing.Value('f', 0)
    
    process_set_speed = multiprocessing.Process(target=set_speed_process, args=[v_desired, w_desired])
    process_set_speed.start()
    
    process_robot_control = multiprocessing.Process(target=robot_control_process, args=[v_desired, w_desired])
    process_robot_control.start()
    
    process_set_speed.join()
    process_robot_control.join()

# No multiprocessing code
def test_no_multiprocess():
    """Testing without multiprocessing
    """
    robot = DiffDriveRobot()
    speed = 0.1
    print(f"Driving forward at {speed} m/s")
    
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=speed, w_desired=0)
        print(f"duty_cycle_L: {duty_cycle_L:5.2f}, duty_cycle_R: {duty_cycle_R:5.2f}, wL_desired: {wL_desired:5.2f}, wL_measured: {wL_measured:5.2f}, wR_desired: {wR_desired:5.2f}, wR_measured: {wR_measured:5.2f}")
        

if __name__ == "__main__":
    # test_no_multiprocess()
    
    test_multiprocess()
