import numpy as np
import math
# from my_visual_kinematics.RobotDelta import RobotDelta
# from my_visual_kinematics.Frame import Frame
# from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from my_visual_kinematics.cascade import *
import matplotlib.pyplot as plt

# Robot Parameters
f = 0.3    # Base equilateral triangle side length
e = 0.1   # End effector equilateral triangle side length
rf = 0.5   # Upper arm length
re = 1   # Lower arm length
tan30 = 1 / np.sqrt(3)
v_max,a_max = 30,30
v_conveyor = 0.3
obj_pos_y = 0.2
duration = 0.25
dt = 0.001

delta_kinematics = DeltaRobotController(f, e, rf, re)
trajectory_generator = TrajectoryGenerator(v_max, a_max,delta_kinematics,v_conveyor,obj_pos_y, duration,dt)
time,poss,vels,accs = trajectory_generator.generate_trapezoidal()

for t in np.arange(0,duration + dt, dt):
    theta_jointvel_set = []
    theta1,theta2,theta3 = delta_kinematics.inverse_kinematics(poss[t][0],poss[t][1],poss[t][2])
    theta1,theta2,theta3, joint_v = delta_kinematics.inverse_kinematics_with_velocity(theta1,theta2,theta3,vels[t][0],vels[t][1],vels[t][2])
    theta_jointvel_set.append([[theta1,theta2,theta3],joint_v])
    print(theta_jointvel_set) 

# # Extract time, positions, velocities, and accelerations from the output
# time = np.array(list(positions.keys()))  # Convert time steps to a numpy array

# # Extract positions
# x_positions = np.array([positions[t][0] for t in time])
# y_positions = np.array([positions[t][1] for t in time])
# z_positions = np.array([positions[t][2] for t in time])

# # Extract velocities
# x_velocities = np.array([velocities[t][0] for t in time])
# y_velocities = np.array([velocities[t][1] for t in time])
# z_velocities = np.array([velocities[t][2] for t in time])

# # Extract accelerations
# x_accelerations = np.array([accelerations[t][0] for t in time])
# y_accelerations = np.array([accelerations[t][1] for t in time])
# z_accelerations = np.array([accelerations[t][2] for t in time])

# # Plot positions over time
# plt.figure(figsize=(10, 6))
# plt.plot(time, x_positions, label="X Position (m)", color='blue')
# plt.plot(time, y_positions, label="Y Position (m)", color='green')
# plt.plot(time, z_positions, label="Z Position (m)", color='red')
# plt.title("Positions Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Position (m)")
# plt.legend()
# plt.grid(True)
# plt.show()

# # Plot velocities over time
# plt.figure(figsize=(10, 6))
# plt.plot(time, x_velocities, label="X Velocity (m/s)", color='blue')
# plt.plot(time, y_velocities, label="Y Velocity (m/s)", color='green')
# plt.plot(time, z_velocities, label="Z Velocity (m/s)", color='red')
# plt.title("Velocities Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (m/s)")
# plt.legend()
# plt.grid(True)
# plt.show()

# # Plot accelerations over time
# plt.figure(figsize=(10, 6))
# plt.plot(time, x_accelerations, label="X Acceleration (m/s²)", color='blue')
# plt.plot(time, y_accelerations, label="Y Acceleration (m/s²)", color='green')
# plt.plot(time, z_accelerations, label="Z Acceleration (m/s²)", color='red')
# plt.title("Accelerations Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Acceleration (m/s²)")
# plt.legend()
# plt.grid(True)
# plt.show()
