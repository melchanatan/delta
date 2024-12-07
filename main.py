import numpy as np
import math
from my_visual_kinematics.RobotDelta import RobotDelta
from my_visual_kinematics.Frame import Frame
from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from my_visual_kinematics.cascade import *
import matplotlib.pyplot as plt
f = 0.16    # Base equilateral triangle side length
e = 0.06   # End effector equilateral triangle side length
rf = 0.3   # Upper arm length
re = 0.5   # Lower arm length
tan30 = 1 / np.sqrt(3)

# Define Delta robot parameters [r1, r2, l1, l2] 
object_pos = [-0.3,-0.2, -0.5]
object_speed = .05
robot = RobotDelta(np.array([f, e, rf, re]), object_pos=object_pos, object_speed=object_speed)

# visualize workspace
# robot.ws_lim = np.array([[-math.pi/12, math.pi/2]]*3)
# robot.ws_division = 10
# robot.show(ws=True)
# print(robot.ws_lim)

x,y,z = x_old,y_old,z_old = 0, 0, -0.4
x_final = object_pos[0] + object_speed*0.25
x_div = (x_final-x)/10
z_div = (z-object_pos[2])/10
control = TrajectoryGenerator(100,50)
_,_,v = control.generate_trapezoidal(np.array([x*10,y*10,z*10]), np.array([x_final*10,y*10,object_pos[2]*10]), duration=0.25)
while True:
    i = 0
    print(v[i])
    i = i+1
    # x = x+x_div # -0.4 to 0.4
    # # y = y+0.1 # -0.3 to 0.3
    # z = z-z_div # -0.5
    # # Define trajectory frames
    # frames = [
    #     Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x_old], [y_old], [z_old]])),
    #     Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]])),
    # ]
    # trajectory = RobotTrajectory(robot, frames)
    
    # trajectory.move_with_speed(speed=round(v[i])*10, motion="p2p", object_speed=robot.object_speed)
                        
    # x_old,y_old,z_old = x,y,z
    # print(x,y,z,v[i])
    





