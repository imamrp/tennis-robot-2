"""
By: Imam Prakoso
This is a redo of the Milestone 1 task of the tennis robot
"""

from robotClasses import DiffDriveRobot
import detection, multiprocessing, time
import numpy as np

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
            
def allign_to_ball(ball_center:int, sum_error:int, desired_center=340, Kp=2e-4, Ki=1e-6, Kd=1e-6):
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

def rotate_robot(w_desired, robot_theta, angle_to_turn, Kp = 6e-1):
    '''
    Rotates the robot angle_to_turn radians on the spot from it's current position

    Args:
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        robot_theta (multiproc variable): Current angle of robot
        angle_to_turn (float): Angle for the robot to turn in radians
        Kp (float): The proportional gain.
    '''
    # finding start theta value as reference
    start_theta = robot_theta.value
    error = angle_to_turn
    while abs(error) > 0.01:
        w_desired.value = error * Kp
        angle_turned = robot_theta.value - start_theta
        error = angle_to_turn - angle_turned
   
    


def move_forward(v_deisred, robot_x, robot_y, dist, Kp = 6e-1):
    '''
    Moves the robot forward dist meters from it's current position

    Args:
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
        robot_x (multiproc variable): Current x-coordinate of robot
        robot_y (multiproc variable): Current y-coordinate of robot
        dist (float): Distance for the robot to move in meters
        Kp (float): The proportional gain.
    '''
    # getting original coodinates as reference
    start_x = robot_x.value
    start_y = robot_y.value
    error = dist
    while abs(error) > 0.01:        # runs until 1cm of target
        v_desired.value = Kp*error
        # finding the distance travelled and comparing to input
        x_travelled = robot_x.value - start_x
        y_travelled = robot_y.value - start_y
        distance_travelled = (x_travelled**2 + y_travelled**2) ** 0.5
        error = dist - distance_travelled
        

def move_to_coord(x_desired, y_desired, robot_x, robot_y, theta, w_desired, v_desired):
    '''
    Moves the robot to any position on the court

    Args:
        x_desired (float): Desired final x coordinate of the robot
        y_desired (float): Desired final y coordinate of the robot
        robot_x (multiproc variable): Current x-coordinate of robot
        robot_y (multiproc variable): Current y-coordinate of robot
        theta (multiproc variable): Current angle of robot
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
    '''
    # clipping theta to [-pi,pi]
    theta_start = (theta.value + np.pi) % (2*np.pi) - np.pi
    angle = np.arctan2((y_desired - robot_y.value), (x_desired - robot_x.value))
    angle_to_turn = angle - theta_start
    # clipping angle to turn to [-pi,pi]
    angle_to_turn = (angle_to_turn + np.pi) % (2*np.pi) - np.pi

    # finding the distance the robot has to move to get to point
    dist = ((x_desired - robot_x.value)**2 + (y_desired - robot_y.value)**2) ** (0.5)
    print('dist: ', dist, 'angle: ', angle_to_turn)

    # turning the robot
    rotate_robot(w_desired = w_desired, robot_theta = theta, angle_to_turn = angle_to_turn)
    print('rotation finished')    
    #calculate turn time
    w_desired.value = 0
    time.sleep(2)

    # moving robot forward
    move_forward(v_deisred = v_desired, robot_x = robot_x, robot_y = robot_y, dist = dist)
    v_desired.value = 0
    time.sleep(2)

    

def milestone1_process(v_desired, w_desired, center, radius, rotbot_x, robot_y, theta):
    """The main process used to control the logic of the robot (e.g. when to align with the ball, setting speed, etc.)

    Args:
        v_desired (float): Velocity of the robot (m/s). Positive is moving forwards.
        w_desired (float): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        center (int): The position of the ball's center in the camera frame. Left of frame is center=0.
        radius (int): Radius of the largest ball in pixels.
        theta (float): Current angle of robot

    Returns:
        None
    """
    print("Milestone 1 process initiated...\n\n\n")
    # '''Stage 1: go to center'''
    # # rotating 90 degrees left test
    # # angle_to_turn = np.pi/2
    # # start_theta = theta.value
    # # error = angle_to_turn
    # # print('rotating robot')
    # # while abs(error) > 0.02:
    # #     error, w_desired.value = rotate_robot(start_theta = start_theta, robot_theta = theta.value, angle_to_turn = angle_to_turn)
    # #     print('w_desired: ', w_desired)
    # #     print('theta: ', theta.value)
    # rotate_robot(w_desired = w_desired, robot_theta = theta, angle_to_turn = np.pi/2)
    # print("Sleep 5 sec")
    # w_desired.value = 0
    # time.sleep(5.0)

    # # rotate_robot(w_desired = w_desired.value, robot_theta = theta.value, angle_to_turn = np.pi/2)
    # # w_desired.value = 0

    # # moving 1 meter forward
    # print('moving forward')
    # move_forward(v_deisred = v_desired, robot_x = robot_x, robot_y = robot_y, dist = 0.3)
    # v_desired.value = 0
    # time.sleep(5.0)
    
    # # returning back to origin
    # print('finished forward movement')
    # move_to_coord(x_desired = 0, y_desired = 0, robot_x = robot_x, robot_y = robot_y, theta = theta, w_desired = w_desired, v_desired = v_desired)
    # print('returned to start')
    
    # '''Stage 2: rotate in place and see a ball'''
    
    
    '''Stage 3: Ball alignment and move towards the ball'''
    alignment_error_sum = 0
    while radius.value < 130:
        v_desired.value = 0.035 # Set slow forward speed
        # Get the desired rotational velocity
        w_desired.value, alignment_error_sum = allign_to_ball(ball_center=center.value, sum_error=alignment_error_sum, desired_center=340)

        if center.value != -1:
            print(f"Target w: {w_desired.value}, Center: {center.value}, Radius: {radius.value}")
            
    # Stop at the ball
    v_desired.value = 0
    w_desired.value = 0
    
    '''Stage 4: Rotate towards start'''
    
    
    '''Stage 5: Traverse to start'''

    
    quit()

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
    robot_x = multiprocessing.Value('f', 0)
    robot_y = multiprocessing.Value('f', 0)
    motor_ctrl_process = multiprocessing.Process(target=robot_control_process, args=(v_desired, w_desired, robot_x, robot_y, theta))
    motor_ctrl_process.start()


    '''Main process'''
    main_process = multiprocessing.Process(target=milestone1_process, args=(v_desired, w_desired, center, radius, robot_x, robot_y, theta))
    main_process.start()

    '''Join processes'''
    motor_ctrl_process.join()
    center_process.join()
    main_process.join()
