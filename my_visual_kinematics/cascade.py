import numpy as np
# from my_visual_kinematics.RobotDelta import RobotDelta
# from my_visual_kinematics.Frame import Frame
# from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from math import pi, atan2, sqrt
import numpy as np

class DeltaRobotController:
    def __init__(self, f, e, rf, re):
        """
        Parameters:
        - f: Base equilateral triangle side length
        - e: End effector equilateral triangle side length
        - rf: Length of the upper arms
        - re: Length of the lower arms
        """
        self.params = [f, e, rf, re]

    @property
    def f(self):
        return self.params[0]

    @property
    def e(self):
        return self.params[1]

    @property
    def rf(self):
        return self.params[2]

    @property
    def re(self):
        return self.params[3]

    # Function for Inverse Kinematics
    def inverse_kinematics(self, x, y, z):
        
        def calculate_angle(x0, y0, z0):
            tan30 = 1 / np.sqrt(3)
            y1 = -0.5 * self.f * tan30
            y0 -= 0.5 * self.e * tan30

            # Intermediate calculations for inverse kinematics
            a = (x0**2 + y0**2 + z0**2 + self.rf**2 - self.re**2 - y1**2) / (2 * z0)
            b = (y1 - y0) / z0

            # Discriminant for quadratic equation
            d = -(a + b * y1)**2 + self.rf * (b**2 * self.rf + self.rf)
            if d < 0:
                raise ValueError(f"Target position ({x0}, {y0}, {z0}) is not reachable.")

            yj = (y1 - a * b - np.sqrt(d)) / (b**2 + 1)
            zj = a + b * yj

            theta = np.arctan(-zj / (y1 - yj))
            return np.degrees(theta)

        # Compute theta1 for the first arm
        theta1 = calculate_angle(x, y, z)

        # Compute theta2 for the second arm (rotated by 120°)
        cos120 = np.cos(2 * np.pi / 3)
        sin120 = np.sin(2 * np.pi / 3)
        x_prime = x * cos120 + y * sin120
        y_prime = y * cos120 - x * sin120
        theta2 = calculate_angle(x_prime, y_prime, z)

        # Compute theta3 for the third arm (rotated by 240°)
        cos240 = np.cos(4 * np.pi / 3)
        sin240 = np.sin(4 * np.pi / 3)
        x_double_prime = x * cos240 + y * sin240
        y_double_prime = y * cos240 - x * sin240
        theta3 = calculate_angle(x_double_prime, y_double_prime, z)

        return theta1, theta2, theta3

    
    def inverse_kinematics_with_velocity(self, x,y,z,vx,vy,vz):
        theta1, theta2, theta3 = self.inverse_kinematics(x,y,z)
        J = self.jacobian(theta1, theta2, theta3)
        cartesian_velocity = np.array([vx,vy,vz])
        joint_velocity = np.linalg.pinv(J).dot(cartesian_velocity)
        return (theta1, theta2, theta3), joint_velocity

    def jacobian(self, theta1, theta2, theta3):
        return np.array([
            [-self.rf * np.sin(np.radians(theta1)), -self.rf * np.sin(np.radians(theta2)), -self.rf * np.sin(np.radians(theta3))],
            [self.rf * np.cos(np.radians(theta1)), self.rf * np.cos(np.radians(theta2)), self.rf * np.cos(np.radians(theta3))],
            [0, 0, 0]
        ])

    

class TrajectoryGenerator:
    def __init__(self, v_max, a_max, dt=0.001):
        self.v_max = v_max  # Maximum velocity
        self.a_max = a_max  # Maximum acceleration
        self.dt = dt        # Time step

    def generate_trapezoidal(self, start_pos, end_pos, duration=0.25, dt=0.01):
        dis = end_pos-start_pos  # Distances for x, y, z axes

        # Initialize variables for each axis
        [x, y, z] = start_pos
        vx, vy, vz = 0, 0, 0
        ax, ay, az = 0, 0, 0

        # Initialize dictionaries to store values for all axes
        s_set, v_set, a_set = {}, {}, {}

        for t in np.arange(0, duration + dt, dt):
            # Acceleration phase
            if t <= duration / 3:
                ax = (2 * dis[0]) / (duration)**2
                ay = (2 * dis[1]) / (duration)**2
                az = (2 * dis[2]) / (duration)**2
                
                vx = ax * t
                vy = ay * t
                vz = az * t
                
                x = x + vx * dt
                y = y + vy * dt
                z = z + vz * dt
            
            # Constant velocity phase
            elif t <= duration * 2 / 3:
                ax, ay, az = 0, 0, 0  
                
                x = x + vx * dt
                y = y + vy * dt
                z = z + vz * dt
            
            # Deceleration phase
            else:
                ax = -(2 * dis[0]) / (duration)**2
                ay = -(2 * dis[1]) / (duration)**2
                az = -(2 * dis[2]) / (duration)**2
                
                vx = ax * (t - duration)
                vy = ay * (t - duration)
                vz = az * (t - duration)
                
                x = x + vx * dt
                y = y + vy * dt
                z = z + vz * dt
            
            # Store values in dictionaries with x, y, z keys
            s_set[t] = {'x': x, 'y': y, 'z': z}
            v_set[t] = {'vx': vx, 'vy': vy, 'vz': vz}
            a_set[t] = {'ax': ax, 'ay': ay, 'az': az}

        # # Print the results
        # print("s_set:", s_set)
        # print("v_set:", v_set)
        # print("a_set:", a_set)
        return t, s_set, v_set, a_set


    

class MotionController:
    def __init__(self, Kp_pos, Kp_vel):
        self.Kp_pos = Kp_pos
        self.Kp_vel = Kp_vel

    def position_control(self, current_pos, target_pos):
        error = target_pos - current_pos
        return self.Kp_pos * error

    def velocity_control(self, v_desired, v_current):
        error = v_desired - v_current
        return self.Kp_vel * error

class DeltaRobotSimulator:
    def __init__(self, kinematics, trajectory_gen):#, motion_controller):
        self.kinematics = kinematics
        self.trajectory_gen = trajectory_gen
        #self.motion_controller = motion_controller

    def simulate_cascade_control(self, start_position, target_position, duration=0.25):
        dt = self.trajectory_gen.dt
        n_steps = int(duration / dt) + 1
        t = np.linspace(0, duration, n_steps)
        
        _, trajectory,_,_ = self.trajectory_gen.generate_trapezoidal(start_position, target_position, duration)
        
        v_cartesian = np.zeros((n_steps, 3))
        v_cartesian[1:] = (trajectory[1:] - trajectory[:-1]) / dt
        
        max_v = 0
        joint_angles = np.zeros((n_steps, 3))
        joint_velocities = np.zeros((n_steps, 3))
        torques = np.zeros((n_steps, 3))
        
        current_position = start_position.copy()
        current_velocity = np.zeros(3)
        
        for i in range(n_steps):
            try:
                angles, velocity = self.kinematics.inverse_kinematics_with_velocity(
                    trajectory[i][0], trajectory[i][1], trajectory[i][2],
                    v_cartesian[i][0], v_cartesian[i][1], v_cartesian[i][2]
                )
                joint_angles[i] = angles
                joint_velocities[i] = velocity
                
                current_v = np.linalg.norm(v_cartesian[i])
                max_v = max(max_v, current_v)
                
                # Instead of position and velocity control, directly update the position and velocity
                if i > 0:
                    current_velocity += torques[i] * dt  # You may update this as needed based on your system dynamics
                    current_position += current_velocity * dt
                    
            except ValueError as e:
                print(f"Step {i}: {e}")
                
        return joint_angles, joint_velocities, torques, t, max_v
