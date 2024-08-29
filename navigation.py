from robotClasses import DiffDriveRobot, RobotController
from matplotlib import pyplot as plt
from IPython import display
import numpy as np
import RPi.GPIO as GPIO

robot = DiffDriveRobot(dt=0.1, wheel_radius=0.028, wheel_sep=0.292)
controller = RobotController(Kp=0.1,Ki=0.01,wheel_radius=0.028,wheel_sep=0.292)

plt.figure(figsize=(15,9))

poses = []
velocities = []
duty_cycle_commands = []
for i in range(100):

    # Example motion using controller 
    
    if i < 50: # drive in circular path (turn left) for 10 s
        duty_cycle_l,duty_cycle_r = controller.drive(0,0.5,robot.wl,robot.wr)
    elif i < 100: # drive in circular path (turn right) for 10 s
        duty_cycle_l,duty_cycle_r = controller.drive(0,-0.5,robot.wl,robot.wr)
    # else: # stop
    #     duty_cycle_l,duty_cycle_r = (0,0)
    
    # Simulate robot motion - send duty cycle command to robot
    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)
    # print(x,y,th)

    #print(duty_cycle_l,duty_cycle_r)
    
    # # Log data
    # poses.append([x,y,th])
    # duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
    # velocities.append([robot.wl,robot.wr])
    
    # # Plot robot data
    # plt.clf()
    # plt.cla()
    # plt.subplot(1,2,1)
    # plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    # plt.plot(x,y,'k',marker='+')
    # plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    # plt.xlim(-1,1)
    # plt.ylim(-1,1)
    # plt.xlabel('x-position (m)')
    # plt.ylabel('y-position (m)')
    # plt.grid()
    
    # plt.subplot(2,2,2)
    # plt.plot(np.arange(i+1)*robot.dt,np.array(duty_cycle_commands))
    # plt.xlabel('Time (s)')
    # plt.ylabel('Duty cycle')
    # plt.grid()
    
    # plt.subplot(2,2,4)
    # plt.plot(np.arange(i+1)*robot.dt,np.array(velocities))
    # plt.xlabel('Time (s)')
    # plt.ylabel('Wheel $\omega$')
    # plt.legend(['Left wheel', 'Right wheel'])
    # plt.grid()
    
    
    # display.clear_output(wait=True)
    # display.display(plt.gcf())

robot.pwm_L.stop()
robot.pwm_R.stop()
#GPIO.cleanup()
