"""
By: Imam Prakoso
This is a redo of the Milestone 1 task of the tennis robot
"""

from robotClasses import DiffDriveRobot


def go_to_ball(center:int, Kp:float, Ki:float, sum_error:int):
    """Function will return a forward velocity and the angular velocity needed to go towards a ball.
    To determine the angular velocity, this function uses a PI controller.

    Args:
        center (int): Horizontal position of the ball in the frame.
        Kp (float): The proportional gain.
        Ki (float): The integral gain.
        sum_error (int): The accumulative sum of the error.

    Returns:
        Tuple(float,float): The desired forward velocity and the desired angular velocity 
    """
    # Set the forward speed
    v_desired = 0.05
    
    # Find error
    error =  - center
    
    
    return v_desired, w_desired, sum_error

def milestone1_process(v_desired,w_desired):
    '''Stage 1: go to center'''
    
    
    '''Stage 2: rotate in place and see a ball'''
    
    
    '''Stage 3: Ball alignment and move towards the ball'''
    
    
    '''Stage 4: Rotate towards start'''
    
    
    '''Stage 5: Traverse to start'''

def robot_control_process(v_desired,w_desired):
    """The process that controls the robot's motors continuously and repeatedly.

    Args:
        v_desired (float): The desired speed of the robot
    """
    robot = DiffDriveRobot()
    
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=v_desired.value, w_desired=w_desired.value)        
    