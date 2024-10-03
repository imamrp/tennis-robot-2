'''
Author: Adam Stecher
Last Modified: 16/09/2024
File contains functions for each state the robot can go to during milestone 2
'''

import numpy as np
import time

### movement functions ###
def move_forward(v_desired, robot_x, robot_y, dist, Kp = 6e-2):
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
    while error > 0.01:        # runs until 1cm of target
        v_desired.value = 0.1
        # finding the distance travelled and comparing to input
        x_travelled = robot_x.value - start_x
        y_travelled = robot_y.value - start_y
        distance_travelled = (x_travelled**2 + y_travelled**2) ** 0.5
        error = dist - distance_travelled
        #print('v desired ', v_desired.value)

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
    move_forward(v_desired = v_desired, robot_x = robot_x, robot_y = robot_y, dist = dist)
    v_desired.value = 0
    time.sleep(2)

### Movement functions end ###

def allign_to_ball(w_desired, v_desired, ball_center, radius, desired_center=220, Kp=2e-4, Ki=1e-6, Kd=1e-6):
    """Function will return a forward velocity and the angular velocity needed to go towards a ball.
    To determine the angular velocity, this function uses a PI controller.

    Args:
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
        ball_center (multiproc variable): Current horizontal position of the ball in the frame.
        radius (multiproc variable): Radius of the largest ball in pixels.
        sum_error (int): The accumulative sum of the error.
        desired_center (int=340): The desired center of the ball (middle pixel on camera)
        Kp (float): The proportional gain.
        Ki (float): The integral gain.

    Returns:
        ball_collected (bool): True if ball is navigated to successfully, False if ball is lost in collection process.
    """
    alignment_error_sum = 0
    counter = 0        # counter for how long the ball is lost for
    ball_collected = False
    while radius.value < 50:
        v_desired.value = 0.035 # Set slow forward speed
        # Get the desired rotational velocity
        # If ball out of frame
        if (ball_center.value==-1):
            w_desired.value = 0
            alignment_error_sum = 0
            time.sleep(0.2)
            counter += 1
            if counter > 20:     # lost for more than 4 seconds
                print('ball no longer found')
                break
        
        # Find error
        error = desired_center - ball_center.value
        
        # PI controller for the desired w (limited to +/-1 rad/s)
        w_desired.value = Kp*error
        
        alignment_error_sum += error
        if ball_center.value != -1:
            print(f"Target w: {w_desired.value}, Center: {ball_center.value}, Radius: {radius.value}")
            lost_counter = 0
    
    if radius.value >= 50:    # ball collected successfully
        ball_collected = True
            
    # Stop at the ball
    v_desired.value = 0
    w_desired.value = 0

    return ball_collected

def line_detector():
    # placeholder until implemented
    return None 

### State Functions ###
def state0(robot_x, robot_y, theta, w_desired, v_desired):
    '''
    Moves to the centre of the court

    Args:
        robot_x (multiproc variable): Current x-coordinate of robot
        robot_y (multiproc variable): Current y-coordinate of robot
        theta (multiproc variable): Current angle of robot
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
    '''
    print('moving to center')
    move_to_coord((5.48/2), (8.23/4), robot_x, robot_y, theta, w_desired, v_desired)

def state1(w_desired, ball_center):
    '''
    Rotate in place and stop when ball is seen
    
    Args:
      w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
      ball_center (multiproc variable): Current horizontal position of the ball in the frame. (-1 if not in frame)
    '''
    # start rotating
    w_desired.value = 0.0    # TODO: test a good search rotation value
    print('rotating until ball found')
    
    # runs until ball seen
    while ball_center.value == -1:
        time.sleep(0.1)
    
    # ball is seen
    w_desired.value = 0
    print('target acquired')

def state2(w_desired, v_desired, ball_center, radius):
    '''
    Moves toward ball until lost or collected

    Args:
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
        ball_center (multiproc variable): Current horizontal position of the ball in the frame.
        radius (multiproc variable): Radius of the largest ball in pixels.

    Returns:
        ball_to_be_collected (bool): True if ball is navigated to successfully, False if ball is lost in collection process.
    '''
    ball_to_be_collected = allign_to_ball(w_desired, v_desired, ball_center, radius)
    print('A ball can be collected: ', ball_to_be_collected)
    return ball_to_be_collected

def state3(v_desired, balls_collected):
    '''
    Jerks robot forward to collect ball

    Args:
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
        balls_collected (int): Amount of balls in the robot's collection

    Returns:
        balls_collected (int): Returns the new amount of balls in the collection after new one
    '''
    # Move forward for a bit
    v_desired.value = 0.1
    time.sleep(3)
    v_desired.value = 0
    print('ball collected')
    return

def state4(robot_x, robot_y, theta, w_desired, v_desired, left_line_detected, right_line_detected):
    '''
    Drive robot to line in the positive y direction (line against courts 2 and 4) and stop when reached

    Args:
        robot_x (multiproc variable): Current x-coordinate of robot
        robot_y (multiproc variable): Current y-coordinate of robot
        theta (multiproc variable): Current angle of robot
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
        left_line_detected (multiproc variable): If the line is detected on the ground from the left RGB sensor.
        right_line_detected (multiproc variable): If the line is detected on the ground from the right RGB sensor.
    '''
    # moving untill 1m away from centre of the line
    move_to_coord((5.48/2), ((8.23/2) - 1), robot_x, robot_y, theta, w_desired, v_desired)
    print('1m away from line')

    # rotating to face line
    desired_rotation = np.pi/2 - theta.value
    rotate_robot(w_desired, robot_theta, angle_to_turn = desired_rotation)
    print('facing the line')

    # moving toward line until detected
    v_desired.value = 0.1
    line_detected = False
    while not line_detected:
        line_detected = left_line_detected.value == 1 or right_line_detected.value == 1
    print('line reached')

    # rotating to face away from the box
    print('rotating on line')
    desired_rotaion = np.pi - theta.value
    rotate_robot(w_desired, robot_theta, angle_to_turn = desired_rotation)
    
def state5(v_desired, w_desired, box_distance, left_line_detected, right_line_detected):
    '''
    follow the line until close to the box

    Args:
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        box_distance (multiproc variable): Distance from the box the ultrasonic sensor detects.
        left_line_detected (multiproc variable): If the line is detected on the ground from the left RGB sensor.
        right_line_detected (multiproc variable): If the line is detected on the ground from the right RGB sensor.
    '''
    print('reversing down the line')
    v_desired.value = -0.05        # TODO: test reversing speed
    while box_distance.value > 10:    # TODO: test distance from box
        # TODO: test alignment method
        if left_line_detected == 1:    # line on left detector
            w_desired.value -= 0.1
        if right_line_detected == 1:
            w_desired.value += 0.1
    print('box reached')
    v_desired.value = 0
    w_desired.value = 0

    # TODO: test update values
    robot_x.value = 5.48 - 0.2
    robot_y.value = 8.23/2
    theta.value = np.pi

def state6(servo):
    '''
    Deposit balls into the box

    Args:
        servo (GPIOzero Servo class): Servo motor to open and close the latch
    '''
    print('opening gate')
    servo.value = -1
    time.sleep(5)
    
    # sleeping to let balls fall out
    print('closing gate')
    servo.value = 1
    

    
    
    
  

