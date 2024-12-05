import numpy as np
from my_visual_kinematics.RobotDelta import RobotDelta
from my_visual_kinematics.Frame import Frame
from my_visual_kinematics.RobotTrajectory import RobotTrajectory

class DeltaRobotController:
    def __init__(self):
        # Robot Parameters
        self.f = 0.3    # Base equilateral triangle side length
        self.e = 0.1    # End effector equilateral triangle side length
        self.rf = 0.6    # Upper arm length
        self.re = 1    # Lower arm length
        self.tan30 = 1 / np.sqrt(3)
        
        # Controller Parameters
        self.Kp_pos = 10  # Proportional gain for position
        self.Kp_vel = 5   # Proportional gain for velocity
        
        # Motion Parameters
        self.v_max = 0.1  # Max velocity (m/s)
        self.a_max = 0.2  # Max acceleration (m/s^2)
        self.dt = 0.01    # Fixed time step at 100Hz
        
        # Initialize robot
        self.object_pos = [-0.3, -0.2, -0.5]
        self.robot = RobotDelta(
            np.array([self.f, self.e, self.rf, self.re]), 
            object_pos=self.object_pos, 
            object_speed=10
        )

class TrajectoryGenerator:
    def __init__(self, v_max, a_max, dt=0.01):
        self.v_max = v_max  # Maximum velocity
        self.a_max = a_max  # Maximum acceleration
        self.dt = dt        # Time step

    def generate_trapezoidal(self, start, end, duration=0.25):
        t = np.arange(0, duration + self.dt, self.dt)  # Time steps
        distance = np.linalg.norm(end - start)         # Total distance
        part_duration = duration / 3                    # Duration for each phase (accel, constant, decel)
        
        t_acc = part_duration                            # Time for acceleration phase
        t_stable = part_duration                        # Time for constant velocity phase
        t_dec = part_duration                            # Time for deceleration phase
        
        v_peak = distance / (t_acc + t_stable + t_dec)   # Peak velocity required to complete trajectory

        # Initialize lists for position and velocity
        trajectory = []
        velocities = []
        
        # Generate trajectory and velocities for each time step
        for time in t:
            if time <= t_acc:  # Acceleration phase
                v = v_peak * (time / t_acc)                               # Linear increase in velocity
                pos = start + 0.5 * v_peak * (time ** 2) / t_acc          # Position increases quadratically
            elif time <= t_acc + t_stable:  # Constant velocity phase
                elapsed_stable = time - t_acc
                pos = start + 0.5 * distance + v_peak * elapsed_stable     # Position increases linearly
                v = v_peak
            else:  # Deceleration phase
                elapsed_decel = time - t_acc - t_stable
                v = v_peak * (1 - elapsed_decel / t_dec)                   # Velocity decreases linearly
                pos = end - 0.5 * v_peak * (elapsed_decel ** 2) / t_dec    # Position decelerates quadratically
            
            trajectory.append(pos)
            velocities.append(v)
        
        # Convert lists to numpy arrays
        trajectory = np.array(trajectory)
        velocities = np.array(velocities)
        
        return t, trajectory

class KinematicsCalculator:
    def __init__(self, f, e, rf, re, tan30):
        self.f = f
        self.e = e
        self.rf = rf
        self.re = re
        self.tan30 = tan30

    def calculate_angle(self, x0, y0, z0):
        y1 = -0.5 * self.f * self.tan30
        y0 -= 0.5 * self.e * self.tan30
        a = (x0**2 + y0**2 + z0**2 + self.rf**2 - self.re**2 - y1**2) / (2 * z0)
        b = (y1 - y0) / z0
        d = -(a + b * y1)**2 + self.rf * (b**2 * self.rf + self.rf)
        
        if d < 0:
            raise ValueError("Target position is not reachable.")
            
        yj = (y1 - a * b - np.sqrt(d)) / (b**2 + 1)
        zj = a + b * yj
        theta = np.arctan(-zj / (y1 - yj))
        return np.degrees(theta)

    def inverse_kinematics_with_velocity(self, x, y, z, vx, vy, vz):
        theta1 = self.calculate_angle(x, y, z)
        
        cos120, sin120 = np.cos(2 * np.pi / 3), np.sin(2 * np.pi / 3)
        cos240, sin240 = np.cos(4 * np.pi / 3), np.sin(4 * np.pi / 3)
        
        x_prime, y_prime = x * cos120 + y * sin120, y * cos120 - x * sin120
        x_double_prime, y_double_prime = x * cos240 + y * sin240, y * cos240 - x * sin240
        
        theta2 = self.calculate_angle(x_prime, y_prime, z)
        theta3 = self.calculate_angle(x_double_prime, y_double_prime, z)
        
        J = self.jacobian(theta1, theta2, theta3)
        cartesian_velocity = np.array([vx, vy, vz])
        joint_velocity = np.linalg.pinv(J).dot(cartesian_velocity)
        
        return (theta1, theta2, theta3), joint_velocity

    def jacobian(self, theta1, theta2, theta3):
        return np.array([
            [-self.rf * np.sin(np.radians(theta1)), -self.rf * np.sin(np.radians(theta2)), -self.rf * np.sin(np.radians(theta3))],
            [self.rf * np.cos(np.radians(theta1)), self.rf * np.cos(np.radians(theta2)), self.rf * np.cos(np.radians(theta3))],
            [0, 0, 0]
        ])

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
        
        _, trajectory = self.trajectory_gen.generate_trapezoidal(start_position, target_position, duration)
        
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