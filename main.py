import numpy as np
import math
from my_visual_kinematics.RobotDelta import RobotDelta
from my_visual_kinematics.Frame import Frame
from my_visual_kinematics.RobotTrajectory import RobotTrajectory

import matplotlib.pyplot as plt
f = 0.16    # Base equilateral triangle side length
e = 0.06   # End effector equilateral triangle side length
rf = 0.3   # Upper arm length
re = 0.5   # Lower arm length
tan30 = 1 / np.sqrt(3)

# Define Delta robot parameters [r1, r2, l1, l2] 
object_pos = [-0.3,-0.2, -0.5]
robot = RobotDelta(np.array([f, e, rf, re]), object_pos=object_pos, object_speed=50)

# visualize workspace
# robot.ws_lim = np.array([[-math.pi/12, math.pi/2]]*3)
# robot.ws_division = 10
# robot.show(ws=True)
# print(robot.ws_lim)

x,y,z =x_old,y_old,z_old =-0.4,-0.2,-0.5
while True:
    x = x+0.1 # -0.4 to 0.4
    # y = y+0.1 # -0.3 to 0.3
    #z = z-0.1 # -0.5
    # Define trajectory frames
    frames = [
        Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x_old], [y_old], [z_old]])),
        Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]])),
    ]
    trajectory = RobotTrajectory(robot, frames)
    
    trajectory.move_with_speed(speed=20, motion="p2p", object_speed=robot.object_speed)
                        
    x_old,y_old,z_old = x,y,z
    # print(x,y,z)
    





