from robotClasses import DiffDriveRobot
import detection, multiprocessing, time

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
def update_ball_center(center, radius): # Takes approximately 0.2s to process one frame
    """Function (or process) that updates the center of the ball and its radius in the camera frame.

    Args:
        center (int): The center of the ball in the frame (340 is exactly center, >340 is to the right of the frame, <340 is the left)
        radius (int): The radius of the largest ball in pixels.

    Returns:
        None
    """
    print("Ball detection process initiated...\n\n\n")
    detector = detection.TennisBallDetector()
    while True:
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

def robot_control_process(v_desired, w_desired, theta):
    """The process that controls the robot's motors continuously and repeatedly.

    Args:
        v_desired (float): The desired speed of the robot
    """
    robot = DiffDriveRobot()
    print("Robot motor control process initiated...\n\n\n")
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=v_desired.value, w_desired=w_desired.value) 
        theta.value = robot.th

def milestone2_process(v_desired, w_desired, center, radius, theta):
    """The process that controls the finite state machine (FSM) for the milestone 2 task. States are numbered with integers as follows:
        State 0: State where the robot is first activated. Transitions: 
        State 1:
        State 2:
        ... TODO: add other states

    Args:
        v_desired (float): The desired speed of the robot
        w_desired (float): The desired rotational speed of the robot speed of the robot
        center (int): The center of the ball in the frame (340 is exactly center, >340 is to the right of the frame, <340 is the left)
        radius (int): The radius of the largest ball in pixels.
        theta (float): The angle of the robot with respect to the starting position. Positive is counter clockwise.
    """

    """Setup"""
    state = 0
    
    """While loop FSM"""
    while True:
        #### State 0: Orient and go to center from start ####
        if state == 0:
            print("State 0: Traversing to center\n\n\n")

            # Orient to center
            
            
            # traverse to center

   
        #### State 1: Find ball by rotating in place ####
        elif state == 1:
            print("State 1: Finding ball\n\n\n")
            while True:
                # Rotate in place
                v_desired.value = 0 
                w_desired.value = 0.2
                
                # Pause if ball seen
                if center.value != -1:
                    w_desired.value = 0
                    time.sleep(2)
                    
                    if center.value != -1: # if ball seen -> state 2
                        state = 2
                        break
                
        
        #### State 2: Align and move towards ball ####
        elif state == 2:
            print("State 2: Aligning with and moving to ball\n\n\n")
            
            # Aligning and going forward
            alignment_error_sum = 0
            while radius.value < 130: 
                v_desired.value = 0.035 # Set slow forward speed
                
                # Get the desired rotational velocity
                w_desired.value, alignment_error_sum = allign_to_ball(ball_center=center.value, 
                                                                      sum_error=alignment_error_sum)

                if center.value != -1: # Lost the ball
                    state = 1
            
            # Ball is close 
            v_desired.value = 0
            w_desired.value = 0
            state = 3
            
        #### State 3: Collecting ball ####
        elif state == 3:
            print("State 3: Collecting ball\n\n\n")
            
            # Move forward for a bit
            v_desired.value = 0.1
            time.sleep(0.5)
            v_desired.value = 0
        
        #### State 4: Drive to lines ####
        elif state == 4:
            print("State 4: Driving to lines\n\n\n")
            
            # Find closest line pointing to the box
            
            # While line NOT detected, drive forwards
            
            # Line detected -> turn away from the box (so that the rear is pointed towards box)
        
        #### State 5: Follow line to the box (driving backwards) ####
        elif state == 5:
            print("State 5: Follow line to box\n\n\n")
            # While ultrasonics read more than a certain length, follow the line and drive backwards
        
        #### State 6: Drop balls and recovery ####
        elif state == 6:
            print("State 6: Dropping balls and recovering\n\n\n")
            # Open hatch
            
            # Drive forward a bit
        
        else:
            print(f"Invalid state: '{state}\n\n\n\'")
            quit()
        
#### Running File ####
if __name__ == "__main__":
    '''Start the update_ball_center process'''
    center = multiprocessing.Value('i', -1)
    radius = multiprocessing.Value('i', -1)
    center_process = multiprocessing.Process(target=update_ball_center, args=(center, radius))
    center_process.start()

    '''Start process for motor control'''
    v_desired = multiprocessing.Value('f', 0)
    w_desired = multiprocessing.Value('f', 0)
    theta = multiprocessing.Value('f', 0)
    motor_ctrl_process = multiprocessing.Process(target=robot_control_process, args=(v_desired, w_desired, theta))
    motor_ctrl_process.start()


    '''Main process'''
    main_process = multiprocessing.Process(target=milestone2_process, args=(v_desired, w_desired, center, radius, theta))
    main_process.start()

    '''Join processes'''
    motor_ctrl_process.join()
    center_process.join()
    main_process.join()
    
