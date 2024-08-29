import time, math
import numpy as np
import RPi.GPIO as GPIO
import gpiozero

class DiffDriveRobot:
    
    def __init__(self, wheel_radius=0.028, wheel_sep=0.292):
        
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        #self.wl = 0.0 #rotational velocity left wheel
        #self.wr = 0.0 #rotational velocity right wheel
        
        #self.dt = dt
        
        self.r = wheel_radius
        self.l = wheel_sep

        self.motor_A_in1 = 19
        self.motor_A_in2 = 26
        self.motor_A_en = 13

        self.motor_B_in2 = 20
        self.motor_B_in1 = 21
        self.motor_B_en = 12

        # Set GPIO modes
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor_A_in1, GPIO.OUT)
        GPIO.setup(self.motor_A_in2, GPIO.OUT)
        GPIO.setup(self.motor_B_in1, GPIO.OUT)
        GPIO.setup(self.motor_B_in2, GPIO.OUT)
        GPIO.setup(self.motor_A_en, GPIO.OUT)
        GPIO.setup(self.motor_B_en, GPIO.OUT)

        # Create PWM instance with a frequency of 100 Hz
        self.pwm_L = GPIO.PWM(self.motor_A_en, 1000)
        self.pwm_R = GPIO.PWM(self.motor_B_en, 1000)

        # Start PWM with a duty cycle of 0%
        self.pwm_L.start(0)
        self.pwm_R.start(0)

        # setting encoders TODO: change to correct encoder output pins
        self.encoderR = gpiozero.RotaryEncoder(a=5, b=6,max_steps=100000)
        self.encoderL = gpiozero.RotaryEncoder(a=2, b=3,max_steps=100000)

    # defining movement functions
    '''
    Function Name: Stop
    stops all movement in the wheels
    Input: None
    Output: None
    '''
    def stop(self):
        GPIO.output(self.motor_B_in1, GPIO.LOW)
        GPIO.output(self.motor_B_in2, GPIO.LOW)
        GPIO.output(self.motor_A_in1, GPIO.LOW)
        GPIO.output(self.motor_A_in2, GPIO.LOW)
        self.pwm_L.ChangeDutyCycle(0)
        self.pwm_R.ChangeDutyCycle(0)
    
    '''
    Function Name: Forward
    Moves the robot forward with a particular duty cycle
    Input: integer from 0 to 100 based on how fast the motors will run
    Output: None
    '''
    def forward(self, duty_cycleL: int, duty_cycleR: int):
        
        self.pwm_L.ChangeDutyCycle(duty_cycleL)
        self.pwm_R.ChangeDutyCycle(duty_cycleR)
        # motor 1 direction
        GPIO.output(self.motor_A_in1, GPIO.HIGH)
        GPIO.output(self.motor_A_in2, GPIO.LOW)
        # motor 2 direction
        GPIO.output(self.motor_B_in1, GPIO.HIGH)
        GPIO.output(self.motor_B_in2, GPIO.LOW)
    
    '''
    Function Name: backward
    Moves the robot backward with a particular duty cycle
    Input: integer from 0 to 100 based on how fast the motors will run
    Output: None
    '''
    def backward(self, duty_cycleL: int, duty_cycleR: int):
        
        self.pwm_L.ChangeDutyCycle(duty_cycleL)
        self.pwm_R.ChangeDutyCycle(duty_cycleR)
        # motor 1 direction
        GPIO.output(self.motor_A_in1, GPIO.LOW)
        GPIO.output(self.motor_A_in2, GPIO.HIGH)
        # motor 2 direction
        GPIO.output(self.motor_B_in1, GPIO.LOW)
        GPIO.output(self.motor_B_in2, GPIO.HIGH)
    
    '''
    Function Name: left
    Moves the robot to the right on the spot with a particular duty cycle
    Input: integer from 0 to 100 based on how fast the motors will run
    Output: None
    '''
    def left(self, duty_cycleL: int, duty_cycleR: int):
        
        self.pwm_L.ChangeDutyCycle(duty_cycleL)
        self.pwm_R.ChangeDutyCycle(duty_cycleR)
        # motor 1 direction
        GPIO.output(self.motor_A_in1, GPIO.HIGH)
        GPIO.output(self.motor_A_in2, GPIO.LOW)
        # motor 2 direction
        GPIO.output(self.motor_B_in1, GPIO.LOW)
        GPIO.output(self.motor_B_in2, GPIO.HIGH)
        
    '''
    Function Name: right
    Moves the robot to the left on the spot with a particular duty cycle
    Input: integer from 0 to 100 based on how fast the motors will run
    Output: None
    '''
    def right(self, duty_cycleL: int, duty_cycleR: int):
        
        self.pwm_L.ChangeDutyCycle(duty_cycleL)
        self.pwm_R.ChangeDutyCycle(duty_cycleR)
        # motor 1 direction
        GPIO.output(self.motor_A_in1, GPIO.LOW)
        GPIO.output(self.motor_A_in2, GPIO.HIGH)
        # motor 2 direction
        GPIO.output(self.motor_B_in1, GPIO.HIGH)
        GPIO.output(self.motor_B_in2, GPIO.LOW)
    
    
    # Kinematic motion model
    def pose_update(self, mode):
        if mode == 'straight':
            # getting distance travelled by each wheel
            dist_R = 2 * self.r * np.pi * -self.encoderR.steps / 900 
            dist_L = 2 * self.r * np.pi * -self.encoderL.steps / 900 
            dist = (dist_R + dist_L) / 2
            delta_th = 0
        else:         # turning
            dist = 0
            delta_th = (self.encoderL.steps - self.encoderR.steps) * (9 / 190)
        
        self.x = self.x + dist * np.cos(self.th)
        self.y = self.y + dist * np.sin(self.th)
        self.th = self.th + delta_th
        self.th = (self.th + 180) % (360) - 180 # clamp theta to [-180, 180]

        self.encoderL.steps = 0
        self.encoderR.steps = 0
        
        return self.x, self.y, self.th

# class RobotController:
    
#     def __init__(self,Kp=0.1,Ki=0.01,wheel_radius=0.028, wheel_sep=0.292):
        
#         self.Kp = Kp
#         self.Ki = Ki
#         self.r = wheel_radius
#         self.l = wheel_sep
#         self.e_sum_l = 0
#         self.e_sum_r = 0
#         self.duty_cycle_l = 0
#         self.duty_cycle_r = 0
        
#     def p_control(self,w_desired,w_measured,e_sum, prev_cycle):
        
#         duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        
#         e_sum = e_sum + (w_desired-w_measured)

#         # duty cycle can only change 0.2 at a time
#         duty_cycle = np.clip(duty_cycle, prev_cycle - 0.2, prev_cycle + 0.2)
        
#         return duty_cycle, e_sum
        
        
#     def drive(self,v_desired,w_desired,wl,wr):
        
#         wl_desired = (v_desired - self.l*w_desired/2)/self.r
#         wr_desired = (v_desired + self.l*w_desired/2)/self.r
#         #print("desired w:", wl_desired, wr_desired)
        
#         duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,wl,self.e_sum_l, self.duty_cycle_l)
#         duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,wr,self.e_sum_r, self.duty_cycle_r)
#         # updating new duty cycles
#         self.duty_cycle_l = duty_cycle_l
#         self.duty_cycle_r = duty_cycle_r
#         #print('sum errors', self.e_sum_l,self.e_sum_r)
        
#         return duty_cycle_l, duty_cycle_r
