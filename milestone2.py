from robotClasses import DiffDriveRobot
import RPi.GPIO as GPIO
import detection, multiprocessing, time, RGBsensor, gpiozero, states

from gpiozero import Servo, PWMOutputDevice
from time import sleep


#### Functions ####
def ctrl_gate(servo, open):
    """Opens and closes gate
    """
    servo.value = -1 if open else 1 # Open gate
    sleep(1)

#### Processes ####
def update_ball_center(center, radius, use_cam): # Takes approximately 0.2s to process one frame
    """Function (or process) that updates the center of the ball and its radius in the camera frame.

    Args:
        center (int): The center of the ball in the frame (340 is exactly center, >340 is to the right of the frame, <340 is the left)
        radius (int): The radius of the largest ball in pixels.
        use_cam (bool): Whether to turn the camera on or off

    Returns:
        None
    """
    print("Ball detection process initiated...\n\n\n")
    detector = detection.TennisBallDetector()
    while True:
        if use_cam.value == 1:
            _, frame = detector.cap.read()
            fH, fW, _ =  frame.shape
            size = 480
            frame = frame[int((fH/2)-size/2):int((fH/2)+size/2), int((fW/2)-size/2):int((fW/2)+size/2)]
            detected_balls = detector.process_frame(frame, False)
            detected_center = detector.get_circle_1_center(detected_balls)
            detected_radius = detector.get_circle_1_radius(detected_balls)
            if detected_center:
                center.value = detected_center[0]
                radius.value = detected_radius
            else:
                center.value = -1
                radius.value = -1
            print(f"X: {center.value} R: {radius.value}")
        else:
            center.value = -1
            radius.value = -1

def sensor_process(box_distance, left_line_detected, right_line_detected):
    """
        This process controls the Ultrasonic and RGB sensors

        Args:
            box_distance (float): distance in cm of object from the ultrasonic sensor
            left_line_detected (bool): If the left rgb sensor detects change in colour in the ground 
            right_line_detected (bool): If the right rgb sensor detects change in colour in the ground 
    """
    TRIG = 10
    ECHO = 9
    dist_sensor = gpiozero.DistanceSensor(echo=ECHO,trigger=TRIG, max_distance=1.0)
    rgb_sensor = RGBsensor.DualSensorReader(threshold = 20)
    GPIO.output(4, GPIO.HIGH)
    while True:
        left_line_detected.value, right_line_detected.value = rgb_sensor.is_line_detected()
        box_distance.value = dist_sensor.distance*100
        time.sleep(0.01)

def robot_control_process(v_desired,w_desired, rotbot_x, robot_y, theta):
    """The process that controls the robot's motors continuously and repeatedly.

    Args:
        v_desired (float): The desired speed of the robot
        w_desired (float): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        robot_x (float): Current x-coordinate of robot
        robot_y (float): Current y-coordinate of robot
        theta (float): Current angle of robot in radians
    """
    robot = DiffDriveRobot()
    print("Robot motor control process initiated...\n\n\n")
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=v_desired.value, w_desired=w_desired.value) 
        # getting current state of robot
        robot_x.value = robot.x
        robot_y.value = robot.y
        theta.value = robot.th

def milestone2_process(v_desired, w_desired, center, radius, rotbot_x, robot_y, theta, use_cam, box_distance, left_line_detected, right_line_detected):
    """The process that controls the finite state machine (FSM) for the milestone 2 task. States are numbered with integers as follows:
        State 0: State where the robot is first activated and moved to the center of the arena. Transitions: State 1
        State 1: State where the robot rotates in place until a ball is located. Transitions: State 2
        State 2: State where the robot aligns with the ball and moves toward it, ready for pickup. Transitions: States 1, 3
        State 3: State where the robot jerks forward to pick up the ball. Transitions: States 1, 4
        State 4: State where the robot drives to the line between court 2 and 4. Transitions: State 5
        State 5: State where the robot reverses along the line utill it is close to the box. Transitions: State 6
        State 6: State where the robot deposits the balls into the box. Transitions: State 0

    Args:
        v_desired (float): The desired speed of the robot
        w_desired (float): The desired rotational speed of the robot speed of the robot
        center (int): The center of the ball in the frame (340 is exactly center, >340 is to the right of the frame, <340 is the left)
        radius (int): The radius of the largest ball in pixels.
        robot_x (float): Current x-coordinate of robot
        robot_y (float): Current y-coordinate of robot
        theta (float): The angle of the robot with respect to the starting position. Positive is counter clockwise.
        use_cam (bool): Whether to turn the camera on or off
        box_distance (float): The distance to the box in cm as measured by ultrasonic sensor
        left_line_detected (bool): True if a line is detected by the LEFT RGB sensor.
        right_line_detected (bool): True if a line is detected by the RIGHT RGB sensor.
    """

    """Setup"""
    state = 1
    balls_collected = 0
    search_direction = 1
    GPIO.setmode(GPIO.BCM)

    # servo setup
    PWM_pin = 11
    servo = Servo(PWM_pin,min_pulse_width=0.002, max_pulse_width=0.0044,frame_width=0.005)    # min and max pulse width may need to be changed if rom not large enough
    ctrl_gate(servo, open=False)
    

    # collector motor setup
    collect_motor = 22        
    GPIO.setup(collect_motor, GPIO.OUT)
    GPIO.output(collect_motor, GPIO.LOW)

    # Eject ball setup (backward motor movement) TODO: implement hardware
    # ejector_motor = 23
    # GPIO.setup(ejector_motor, GPIO.OUT)
    # GPIO.output(ejector_motor, GPIO.LOW)

    print('startup')
    time.sleep(1)
    print('startup.')
    time.sleep(1)
    print('startup..')
    time.sleep(1)
    print('startup...\n\n\n\n\n')
    time.sleep(1)
    print(f'starting in state {state}') 
    
    """While loop FSM"""
    while True:
        #### State 0: Orient and go to center from start ####
        if state == 0:
            print("State 0: Traversing to center==============================\n\n\n")
            states.state0(robot_x, robot_y, theta, w_desired, v_desired)
            # Orient to center
            state = 1
            
            # traverse to center

   
        #### State 1: Find ball by rotating in place ####
        elif state == 1:
            print("State 1: Finding ball====================================\n\n\n")
            use_cam.value = 1
            print('camera activated: ',use_cam.value)
            states.state1(w_desired, center, search_direction)
            state = 2
        
        #### State 2: Align and move towards ball ####
        elif state == 2:
            print("State 2: Aligning with and moving to ball====================\n\n\n")
            use_cam.value = 1
            to_collect = states.state2(w_desired, v_desired, center, radius)
            if to_collect:
                state = 3
                use_cam.value = 0
            else:
                state = 1
                search_direction = search_direction * -1
            
        #### State 3: Collecting ball ####
        elif state == 3:
            print("State 3: Collecting ball====================================\n\n\n")
            # starting motor
            GPIO.output(collect_motor, GPIO.HIGH)
            time.sleep(3)        # TODO: confirm if good amount of sleep time
            
            states.state3(v_desired, balls_collected)
            balls_collected += 1
            if balls_collected >= 3:
                state = 4
            else:
                state = 1

            # turning motor off
            GPIO.output(collect_motor, GPIO.LOW)

            # rotating motor other way to make sure ball doesnt stay stuck
            # time.sleep(3)
            # GPIO.output(ejector_motor, GPIO.HIGH)
            # time.sleep(3)        # TODO: confirm if good amount of sleep time
            # GPIO.output(ejector_motor, GPIO.LOW)
        
        #### State 4: Drive to lines ####
        elif state == 4:
            print("State 4: Driving to lines====================================\n\n\n")
            states.state4(robot_x, robot_y, theta, w_desired, v_desired, left_line_detected, right_line_detected)
            state = 5
        
        #### State 5: Follow line to the box (driving backwards) ####
        elif state == 5:
            print("State 5: Follow line to box=====================================\n\n\n")
            states.state5(robot_x, robot_y, theta, v_desired, w_desired, box_distance, left_line_detected, right_line_detected)
            state = 6
            # While ultrasonics read more than a certain length, follow the line and drive backwards
        
        #### State 6: Drop balls and recovery ####
        elif state == 6:
            print("State 6: Dropping balls and recovering==============================\n\n\n")
            states.state6(servo)
            state = 0
            # Open hatch
            balls_collected = 0 
            
            # Drive forward a bit
        
        else:
            print(f"Invalid state: '{state}'==============================================\n\n\n")
            quit()
        
#### Running File ####
if __name__ == "__main__":
    '''Start the update_ball_center process'''
    use_cam = multiprocessing.Value('i', 0)
    center = multiprocessing.Value('i', -1)
    radius = multiprocessing.Value('i', -1)
    center_process = multiprocessing.Process(target=update_ball_center, args=(center, radius, use_cam))
    center_process.start()

    '''Start process for motor control'''
    v_desired = multiprocessing.Value('f', 0)
    w_desired = multiprocessing.Value('f', 0)
    theta = multiprocessing.Value('f', 0)
    robot_x = multiprocessing.Value('f', 0)
    robot_y = multiprocessing.Value('f', 0)
    motor_ctrl_process = multiprocessing.Process(target=robot_control_process, args=(v_desired, w_desired, robot_x, robot_y, theta))
    motor_ctrl_process.start()

    '''Sensor Process'''
    box_distance = multiprocessing.Value('f', 0)
    left_line_detected = multiprocessing.Value('b', False)
    right_line_detected = multiprocessing.Value('b', False)
    sensor_process = multiprocessing.Process(target=sensor_process, args=(box_distance, left_line_detected, right_line_detected))
    sensor_process.start()
    
    '''Main process'''
    main_process = multiprocessing.Process(target=milestone2_process, args=(v_desired, w_desired, center, radius, robot_x, robot_y, theta, use_cam, box_distance, left_line_detected, right_line_detected))
    main_process.start()

    '''Join processes'''
    motor_ctrl_process.join()
    center_process.join()
    main_process.join()
    sensor_process.join()
    
