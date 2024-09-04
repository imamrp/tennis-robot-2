"""
By: Imam Prakoso
This is a redo of the Milestone 1 task of the tennis robot
"""

from robotClasses import DiffDriveRobot


def allign_to_ball(ball_center:int, sum_error:int, desired_center=340, Kp=0.1, Ki=0.01):
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
    
    # Find error
    error = desired_center - ball_center
    
    # PI controller for the desired w (limited to +/-1 rad/s)
    w_desired = min(max(-1,Kp*error + Ki*sum_error),1)
    
    sum_error += error
    
    return w_desired, sum_error

def milestone1_process(v_desired,w_desired):
    '''Stage 1: go to center'''
    
    
    '''Stage 2: rotate in place and see a ball'''
    
    
    '''Stage 3: Ball alignment and move towards the ball'''
    
    
    '''Stage 4: Rotate towards start'''
    
    
    '''Stage 5: Traverse to start'''
    pass

def robot_control_process(v_desired,w_desired):
    """The process that controls the robot's motors continuously and repeatedly.

    Args:
        v_desired (float): The desired speed of the robot
    """
    robot = DiffDriveRobot()
    
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=v_desired.value, w_desired=w_desired.value)        


if __name__ == "__main__":
    