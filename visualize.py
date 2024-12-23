import numpy as np
import math
# from my_visual_kinematics.RobotDelta import RobotDelta
# from my_visual_kinematics.Frame import Frame
# from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from my_visual_kinematics.cascade import *
import matplotlib.pyplot as plt


def generate_trajectory(v_conveyor, obj_pos_y, duration, graph=False):
   # Robot Parameters
    f = 0.5    # Base equilateral triangle side length
    e = 0.045   # End effector equilateral triangle side length
    rf = 0.3   # Upper arm length
    re = 0.7   # Lower arm length
    conveyor_lenght = 3.5
    mass = 0.5
    dt = 0.01
    joint_set = []

    tan30 = 1 / np.sqrt(3)
    delta_kinematics = DeltaRobotController(f, e, rf, re, mass)
    trajectory_generator = TrajectoryGenerator(
        delta_kinematics, v_conveyor, conveyor_lenght, obj_pos_y, duration, dt)
    offset_conveyor_upperbase = delta_kinematics.calculate_middle_taskspace()
    time, poss, vels, accs = trajectory_generator.generate_trapezoidal()
    wait_for_object = conveyor_lenght/v_conveyor*0.5
    print(offset_conveyor_upperbase)
    for t in np.arange(0, duration + dt, dt):
        theta1, theta2, theta3 = delta_kinematics.inverse_kinematics(
            poss[t][0], poss[t][1], poss[t][2])
        theta1, theta2, theta3, joint_v = delta_kinematics.inverse_kinematics_with_velocity(
            theta1, theta2, theta3, vels[t][0], vels[t][1], vels[t][2])
        torque = delta_kinematics.calculate_motor_torques_yz(
            theta1, theta2, theta3)
        joint_set.append([[theta1, theta2, theta3], joint_v, torque, t])
    if graph:
        # Print tarjectory graph
        # Convert time steps to a numpy array
        time = np.array(list(poss.keys()))

        # Extract positions
        x_positions = np.array([poss[t][0] for t in time])
        y_positions = np.array([poss[t][1] for t in time])
        z_positions = np.array([poss[t][2] for t in time])

        # Extract velocities
        x_velocities = np.array([vels[t][0] for t in time])
        y_velocities = np.array([vels[t][1] for t in time])
        z_velocities = np.array([vels[t][2] for t in time])

        # Extract accelerations
        x_accelerations = np.array([accs[t][0] for t in time])
        y_accelerations = np.array([accs[t][1] for t in time])
        z_accelerations = np.array([accs[t][2] for t in time])

        # Plot positions over time
        plt.figure(figsize=(10, 6))
        plt.plot(time, x_positions, label="X Position (m)", color='blue')
        plt.plot(time, y_positions, label="Y Position (m)", color='green')
        plt.plot(time, z_positions, label="Z Position (m)", color='red')
        plt.title("Positions Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
        plt.grid(True)
        plt.show()

        # Plot velocities over time
        plt.figure(figsize=(10, 6))
        plt.plot(time, x_velocities, label="X Velocity (m/s)", color='blue')
        plt.plot(time, y_velocities, label="Y Velocity (m/s)", color='green')
        plt.plot(time, z_velocities, label="Z Velocity (m/s)", color='red')
        plt.title("Velocities Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.grid(True)
        plt.show()

        # Plot accelerations over time
        plt.figure(figsize=(10, 6))
        plt.plot(time, x_accelerations,
                 label="X Acceleration (m/s²)", color='blue')
        plt.plot(time, y_accelerations,
                 label="Y Acceleration (m/s²)", color='green')
        plt.plot(time, z_accelerations,
                 label="Z Acceleration (m/s²)", color='red')
        plt.title("Accelerations Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (m/s²)")
        plt.legend()
        plt.grid(True)
        plt.show()

    return {"trajectory": joint_set, "wait_time": wait_for_object}


if __name__ == "__main__":
    generate_trajectory(0.2, 1, 0.25)

    # print(joint_v)
    # print(theta1,theta2,theta3,t)
    # print
