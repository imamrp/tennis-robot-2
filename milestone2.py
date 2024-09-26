from robotClasses import DiffDriveRobot
import RPi.GPIO as GPIO
import detection, multiprocessing, time, RGBsensor, gpiozero, states

#### Functions ####
def allign_to_ball(ball_center:int, sum_error:int, desired_center=340, Kp=6e-4, Ki=1e-3):
    """Function will return a forward velocity and the angular velocity needed to go towards a ball.
    To determine the angular velocity, this function uses a PI controller.

    Args:
        ball_center (int): Current horizontal position of the ball in the frame.
        sum_error (int): The accumulative sum of the error.
        desired_center (int=340): The desired center of the ball
        Kp (float): The proportional gain.
        Ki (float): The integral gain.

    Returns:
        Tuple(float,float): The desired forward velocity and the desired angular velocity 
    """
    # If ball out of frame
    if (ball_center==-1):
        w_desired = 0
        sum_error = 0
        return w_desired, sum_error
    
    # Find error
    error = desired_center - ball_center
    
    # PI controller for the desired w (limited to +/-1 rad/s)
    w_desired = Kp*error
    
    sum_error += error
    return w_desired, sum_error

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
        if use_cam.value is True:
            _, frame = detector.cap.read()
            detected_balls = detector.process_frame(frame)
            detected_center = detector.get_circle_1_center(detected_balls)
            detected_radius = detector.get_circle_1_radius(detected_balls)
            if detected_center:
                center.value = detected_center[0]
                radius.value = detected_radius
            else:
                center.value = -1
                radius.value = -1
            # print(f"X: {center.value} R: {radius.value}")
        else:
            center.value = -1
            radius.value = -1

def sensor_process():
    """
        This process controls the Ultrasonic and RGB sensors
    """
    TRIG = 10
    ECHO = 9
    dist_sensor = gpiozero.DistanceSensor(echo=ECHO,trigger=TRIG, max_distance=1.0)
    rgb_sensor = RGBsensor.DualSensorReader(4)
    GPIO.output(4, GPIO.HIGH)
    while True:
        print(rgb_sensor.is_line_detected())
        print('Distance: ', dist_sensor.distance * 100)
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

def milestone2_process(v_desired, w_desired, center, radius, rotbot_x, robot_y, theta, use_cam):
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
    """

    """Setup"""
    state = 0
    balls_collected = 0
    
    """While loop FSM"""
    while True:
        #### State 0: Orient and go to center from start ####
        if state == 0:
            print("State 0: Traversing to center\n\n\n")
            states.state0(robot_x, robot_y, theta, w_desired, v_desired)
            # Orient to center
            state = 1
            
            # traverse to center

   
        #### State 1: Find ball by rotating in place ####
        elif state == 1:
            print("State 1: Finding ball\n\n\n")
            use_cam.value = True
            states.state1(w_desired, center)
            state = 2
        
        #### State 2: Align and move towards ball ####
        elif state == 2:
            print("State 2: Aligning with and moving to ball\n\n\n")
            to_collect = states.state2(w_desired, v_desired, center, radius)
            if to_collect:
                state = 3
            else:
                state = 1
            
        #### State 3: Collecting ball ####
        elif state == 3:
            print("State 3: Collecting ball\n\n\n")
            balls_collected = states.state3(v_desired, balls_collected)
            if balls_collected >= 3:
                state = 4
            else:
                state = 1
        
        #### State 4: Drive to lines ####
        elif state == 4:
            print("State 4: Driving to lines\n\n\n")
            states.state4(x_desired, y_desired, robot_x, robot_y, theta, w_desired, v_desired)
            state = 5
        
        #### State 5: Follow line to the box (driving backwards) ####
        elif state == 5:
            print("State 5: Follow line to box\n\n\n")
            states.state5(v_desired, w_desired, ultrasonic_dist)
            state = 6
            # While ultrasonics read more than a certain length, follow the line and drive backwards
        
        #### State 6: Drop balls and recovery ####
        elif state == 6:
            print("State 6: Dropping balls and recovering\n\n\n")
            states.state6()
            state = 0
            # Open hatch
            
            # Drive forward a bit
        
        else:
            print(f"Invalid state: '{state}\n\n\n\'")
            quit()
        
#### Running File ####
if __name__ == "__main__":
    '''Start the update_ball_center process'''
    use_cam = multiprocessing.Value('b', False)
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


    '''Main process'''
    main_process = multiprocessing.Process(target=milestone2_process, args=(v_desired, w_desired, center, radius, robot_x, robot_y, theta, use_cam))
    main_process.start()

    '''Sensor Process'''
    sensor_process = multiprocessing.Process(target=sensor_process)
    sensor_process.start()

    '''Join processes'''
    motor_ctrl_process.join()
    center_process.join()
    main_process.join()
    sensor_process.join()
    
