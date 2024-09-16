'''
Author: Adam Stecher
Last Modified: 16/09/2024
File contains functions for each state the robot can go to during milestone 2
'''

import numpy as np
import time

### movement functions ###
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

### Movement functions end ###

def allign_to_ball(w_desired, v_desired, ball_center, radius, sum_error:int, desired_center=340, Kp=2e-4, Ki=1e-6, Kd=1e-6):
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
    """
    alignment_error_sum = 0
    while radius.value < 130:
        v_desired.value = 0.035 # Set slow forward speed
        # Get the desired rotational velocity
        # If ball out of frame
        if (ball_center==-1):
            w_desired.value = 0
            alignment_error_sum = 0
        
        # Find error
        error = desired_center - ball_center.value
        
        # PI controller for the desired w (limited to +/-1 rad/s)
        w_desired.value = Kp*error
        
        alignment_error_sum += error
        if center.value != -1:
            print(f"Target w: {w_desired.value}, Center: {center.value}, Radius: {radius.value}")
            
    # Stop at the ball
    v_desired.value = 0
    w_desired.value = 0

### State Functions ###
def state0(x_desired, y_desired, robot_x, robot_y, theta, w_desired, v_desired)
  '''
    Moves to the centre of the court

    Args:
        x_desired (float): Desired final x coordinate of the robot
        y_desired (float): Desired final y coordinate of the robot
        robot_x (multiproc variable): Current x-coordinate of robot
        robot_y (multiproc variable): Current y-coordinate of robot
        theta (multiproc variable): Current angle of robot
        w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
        v_desired (multiproc variable): Linear velocity of the robot (m/s).
    '''
      move_to_coord(x_desired = 5.48/2, y_desired = 8.23/4, robot_x, robot_y, theta, w_desired, v_desired)

def state1(w_desired, ball_center):
  '''
  Rotate in place and stop when ball is seen
  
  Args:
      w_desired (multiproc variable): Angular velocity of the robot (rad/s). Positive is moving counter clockwise (left).
      ball_center (multiproc variable): Current horizontal position of the ball in the frame. (-1 if not in frame)
  '''
    # start rotating
    w_desired.value = 0.05    # TODO: test a good search rotation value

    # runs until ball seen
    while ball_centre.value == -1:
        time.sleep(0.1)

    # ball is seen
    w_desired.value = 0
  

