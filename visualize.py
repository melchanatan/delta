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

# Define positions
start_position = np.array([0.0, 0.0, -0.5])
target_position = np.array([0.2, 0.2, -0.6])

delta_kinematics = DeltaRobotController(f, e, rf, re)
trajectory_generator = TrajectoryGenerator(v_max, a_max, dt=0.01)
t,positions,velocities,accelerations = trajectory_generator.generate_trapezoidal(start_position,target_position)
delta_simulator = DeltaRobotSimulator(delta_kinematics, trajectory_generator)


# Run simulation
duration = 0.25
joint_angles, joint_velocities, torques, time, max_velocity = delta_simulator.simulate_cascade_control(
    start_position, target_position, duration
)

print("Time Steps:", time)
print("Positions (x, y, z):", positions)
print("Velocities (vx, vy, vz):", velocities)

# Plot results
import matplotlib.pyplot as plt

# Plot positions
plt.figure(figsize=(10, 6))
plt.plot(time, positions[:, 0], label="X Position (m)", color='blue')
plt.plot(time, positions[:, 1], label="Y Position (m)", color='green')
plt.plot(time, positions[:, 2], label="Z Position (m)", color='red')
plt.title("Positions Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid(True)
plt.show()

# Plot velocities
plt.figure(figsize=(10, 6))
plt.plot(time, velocities[:, 0], label="X Velocity (m/s)", color='blue')
plt.plot(time, velocities[:, 1], label="Y Velocity (m/s)", color='green')
plt.plot(time, velocities[:, 2], label="Z Velocity (m/s)", color='red')
plt.title("Velocities Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.grid(True)
plt.show()


# # Plot Joint Angles vs Time
# plt.figure(figsize=(10, 6))
# plt.plot(time, joint_angles[:, 0], label="Theta1 (°)", color='blue')
# plt.plot(time, joint_angles[:, 1], label="Theta2 (°)", color='green')
# plt.plot(time, joint_angles[:, 2], label="Theta3 (°)", color='red')
# plt.title("Joint Angles Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Joint Angles (degrees)")
# plt.legend()
# plt.grid(True)

# # Plot Joint Velocities vs Time
# plt.figure(figsize=(10, 6))
# plt.plot(time, joint_velocities[:, 0], label="Velocity1 (rad/s)", color='blue')
# plt.plot(time, joint_velocities[:, 1], label="Velocity2 (rad/s)", color='green')
# plt.plot(time, joint_velocities[:, 2], label="Velocity3 (rad/s)", color='red')
# plt.title("Joint Velocities Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Joint Velocities (rad/s)")
# plt.legend()
# plt.grid(True)

# # Plot Torques vs Time
# plt.figure(figsize=(10, 6))
# plt.plot(time, torques[:, 0], label="Torque1 (Nm)", color='blue')
# plt.plot(time, torques[:, 1], label="Torque2 (Nm)", color='green')
# plt.plot(time, torques[:, 2], label="Torque3 (Nm)", color='red')
# plt.title("Torques Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Torques (Nm)")
# plt.legend()
# plt.grid(True)

# plt.show()
