import numpy as np
import math
from my_visual_kinematics.RobotDelta import RobotDelta
from my_visual_kinematics.Frame import Frame
from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from my_visual_kinematics.robot_controller import *
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Initialize components
    robot_controller = DeltaRobotController()
    kinematics = KinematicsCalculator(
        robot_controller.f, 
        robot_controller.e, 
        robot_controller.rf, 
        robot_controller.re, 
        robot_controller.tan30
    )
    trajectory_gen = TrajectoryGenerator(
        robot_controller.v_max, 
        robot_controller.a_max
    )
    motion_controller = MotionController(
        robot_controller.Kp_pos, 
        robot_controller.Kp_vel
    )
    simulator = DeltaRobotSimulator(
        kinematics, 
        trajectory_gen, 
        # motion_controller
    )

    # Initial position
    x, y, z = x_old, y_old, z_old = -0.4, -0.2, -0.5

    # Main control loop
    while True:
        x = x + 0.4
        
        print("x: ", x )# -0.4 to 0.4
        print("y: ", y)
        print("z: ", z)

         
        # Define trajectory frames
        frames = [
            Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x_old], [y_old], [z_old]])),
            Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]])),
        ]
        trajectory = RobotTrajectory(robot_controller.robot, frames)
        
        # Simulate motion
        joint_angles, joint_velocities, torques, t, v = simulator.simulate_cascade_control(
            np.array([x, y, z]),
            np.array(robot_controller.object_pos)
        )
        
        # Execute motion
        trajectory.move_with_speed(
            speed=round(v), 
            motion="p2p", 
            object_speed=robot_controller.robot.object_speed
        )
        
        x_old, y_old, z_old = x, y, z