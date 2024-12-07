import numpy as np
import math
# from my_visual_kinematics.RobotDelta import RobotDelta
# from my_visual_kinematics.Frame import Frame
# from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from my_visual_kinematics.cascade import *

import matplotlib.pyplot as plt

# Robot Parameters
f = 0.16    # Base equilateral triangle side length
e = 0.06   # End effector equilateral triangle side length
rf = 0.3   # Upper arm length
re = 0.5   # Lower arm length
tan30 = 1 / np.sqrt(3)
v_max,a_max = 0.5,0.5

robot = DeltaRobotController(f,e,rf,re)
theta = robot.inverse(np.array([0,0,-0.5]))
traject = TrajectoryGenerator(v_max,a_max)
traject.generate_trapezoidal(np.array([0,0,-0.4]),np.array([0,0,-0.6]))
print(theta)
# q1,q2,q3 = delta.inverse(0.0,0.0,-0.4)
# print(q1,q2,q3)