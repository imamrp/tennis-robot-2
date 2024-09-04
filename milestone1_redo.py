"""
By: Imam Prakoso
This is a redo of the Milestone 1 task of the tennis robot
"""

from robotClasses import DiffDriveRobot
import detection, multiprocessing, time

def update_ball_center(center, radius): # Takes approximately 0.2s to process one frame
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
            
def allign_to_ball(ball_center:int, sum_error:int, desired_center=340, Kp=1e-6, Ki=0):
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
        return w_desired, sum_error
    
    # Find error
    error = desired_center - ball_center
    
    # PI controller for the desired w (limited to +/-1 rad/s)
    w_desired = min(max(-0.5,Kp*error + Ki*sum_error),0.5)
    
    sum_error += error
    
    return w_desired, sum_error

def milestone1_process(v_desired, w_desired, center, radius):
    print("Milestone 1 process initiated...\n\n\n")
    '''Stage 1: go to center'''
    
    
    '''Stage 2: rotate in place and see a ball'''
    
    
    '''Stage 3: Ball alignment and move towards the ball'''
    alignment_error_sum = 0
    while radius.value < 130:
        v_desired.value = 0 # Set slow forward speed
        # Get the desired rotational velocity
        w_desired.value, alignment_error_sum = allign_to_ball(ball_center=center.value, sum_error=alignment_error_sum, desired_center=340, Kp=0.1, Ki=0.01)

        if center.value != -1:
            print(f"Target w: {w_desired.value}, Center: {center.value}, Radius: {radius.value}")

    # Stop at the ball
    v_desired.value = 0
    w_desired.value = 0
    
    '''Stage 4: Rotate towards start'''
    
    
    '''Stage 5: Traverse to start'''

    
    quit()

def robot_control_process(v_desired,w_desired):
    """The process that controls the robot's motors continuously and repeatedly.

    Args:
        v_desired (float): The desired speed of the robot
    """
    robot = DiffDriveRobot()
    print("Robot motor control process initiated...\n\n\n")
    while True:
        duty_cycle_L, duty_cycle_R, wL_desired, wL_measured, wR_desired, wR_measured = robot.drive(v_desired=v_desired.value, w_desired=w_desired.value)        

if __name__ == "__main__":
    '''Start the update_ball_center process'''
    center = multiprocessing.Value('i', -1)
    radius = multiprocessing.Value('i', -1)
    center_process = multiprocessing.Process(target=update_ball_center, args=(center, radius))
    center_process.start()

    '''Start process for motor control'''
    v_desired = multiprocessing.Value('f', 0)
    w_desired = multiprocessing.Value('f', 0)
    motor_ctrl_process = multiprocessing.Process(target=robot_control_process, args=(v_desired, w_desired))
    motor_ctrl_process.start()


    '''Main process'''
    main_process = multiprocessing.Process(target=milestone1_process, args=(v_desired, w_desired, center, radius))
    main_process.start()

    '''Join processes'''
    motor_ctrl_process.join()
    center_process.join()
    main_process.join()
