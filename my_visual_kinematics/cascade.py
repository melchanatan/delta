import numpy as np
# from my_visual_kinematics.RobotDelta import RobotDelta
# from my_visual_kinematics.Frame import Frame
# from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from math import pi, atan2, sqrt, sin, cos, radians

class DeltaRobotController:
    def __init__(self, f, e, rf, re,mass):
        """
        Parameters:
        - f: Base equilateral triangle side length
        - e: End effector equilateral triangle side length
        - rf: Length of the upper arms
        - re: Length of the lower arms
        """
        self.params = [f, e, rf, re]
        self.mass = mass

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
        return theta1,theta2,theta3

    def convert_to_python(self,data):
        """
        Recursively convert numpy data types and arrays to Python native types.
        """
        if isinstance(data, np.ndarray):  # If it's a numpy array, convert to a list
            return data.tolist()
        elif isinstance(data, np.generic):  # If it's a numpy scalar, convert to a Python scalar
            return data.item()
        elif isinstance(data, list):  # If it's a list, recursively process its elements
            return [self.convert_to_python(item) for item in data]
        elif isinstance(data, tuple):  # If it's a tuple, recursively process and return as a tuple
            return tuple(self.convert_to_python(item) for item in data)
        else:  # If it's any other type, return as is
            return data

    def inverse_kinematics_with_velocity(self, theta1, theta2, theta3, vx, vy, vz):
        J = self.jacobian(theta1, theta2, theta3)
        cartesian_velocity = np.array([vx, vy, vz])
        joint_velocity = np.linalg.pinv(J).dot(cartesian_velocity)

        # Convert results to Python native types
        theta1 = self.convert_to_python(theta1)
        theta2 = self.convert_to_python(theta2)
        theta3 = self.convert_to_python(theta3)
        joint_velocity = self.convert_to_python(joint_velocity)

        return theta1, theta2, theta3, joint_velocity
    def calculate_motor_torques_yz(self,theta1,theta2,theta3): #F = [Fx,Fy,Fz] theta = [theta1,theta2,theta3]
        # Convert angles from degrees to radians
        theta_rad = [radians(angle) for angle in [theta1,theta2,theta3]]
        theta1, theta2, theta3 = theta_rad

        # Compute Jacobian for the YZ plane
        J = self.jacobian(theta1, theta2, theta3)
        F = [0,0,self.mass]
        # Compute torques using the Jacobian transpose
        tau = np.dot(J.T, F)  # Torque vector
        return tau.tolist()
    
    def jacobian(self, theta1, theta2, theta3):
        return np.array([
            [0,0,0],
            [-self.rf * np.sin(np.radians(theta1)), -self.rf * np.sin(np.radians(theta2)), -self.rf * np.sin(np.radians(theta3))],
            [self.rf * np.cos(np.radians(theta1)), self.rf * np.cos(np.radians(theta2)), self.rf * np.cos(np.radians(theta3))] 
        ])
        
    def calculate_homeconfig_pos(self): #homeconfig_pos of end_effector
        if(self.re < self.rf+(self.f/(2 * np.sqrt(3)))-(self.e/(2 * np.sqrt(3)))):
            raise Exception("Lower arm is too SHORT")
        
        tan30 = 1 / np.sqrt(3)  # tan(30Â°)
        
        # Distance in the XY-plane from the origin to the end-effector center
        xy_distance = self.rf + (self.f * tan30) - (self.e * tan30)
        print("xy_distance = ",xy_distance)
        # Solve for Z using Pythagorean theorem
        z = -np.sqrt(self.re**2 - xy_distance**2)
        
        # At the initial state, the end effector is aligned along the Z-axis
        x = 0.0
        y = 0.0
        print("homeconfig position = ",[x,y,z])
        return [x, y, z]
    
    def calculate_lowest_z(self):
        if((self.f/(2 * np.sqrt(3))) > self.re + self.rf + (self.e/(2 * np.sqrt(3)))):
            raise Exception("ERROR: Base is too LONG.")
        # Offset from the triangular base and end effector
        offset = 0.5 * (self.f / np.sqrt(3) - self.e / np.sqrt(3))
        
        # Total length when fully stretched
        total_length = self.rf + self.re
        
        # Vertical projection for z using the total length and offset
        lowest_z = -np.sqrt(total_length**2 - offset**2)
        
        print("lowest_z = ",lowest_z)
        # Return the position tuple
        return lowest_z
    
    def calculate_middle_taskspace(self):
        z_min = self.calculate_homeconfig_pos()[2]
        z_max = self.calculate_lowest_z()
        middle_taskspace = (z_min+z_max)/2
        print("middle_taskspace = ",middle_taskspace)
        return middle_taskspace #z-axis

class TrajectoryGenerator:
    def __init__(self, v_max, a_max,delta_robot,v_conveyor,obj_pos_y, duration=0.25,dt=0.01):
        self.v_max = v_max  # Maximum velocity
        self.a_max = a_max  # Maximum acceleration
        self.obj_pos_y = obj_pos_y # start position in y axis of object
        self.delta_robot = delta_robot
        self.v_conveyor = v_conveyor
        self.duration = duration
        self.dt = dt        # Time step
    
    def end_position(self): #obj_start_pos = y axis
        obj_end_pos = [0,0,0]
        obj_end_pos[0] = self.v_conveyor * self.duration #x axis
        obj_end_pos[1] = self.obj_pos_y
        obj_end_pos[2] = self.delta_robot.calculate_middle_taskspace() #z axis
        print("obj_end_pos = ",obj_end_pos)
        return obj_end_pos
    
    def generate_trapezoidal(self):
        start_pos = np.array(self.delta_robot.calculate_homeconfig_pos())  # Convert to NumPy array
        end_pos = np.array(self.end_position(), dtype=float)  # Convert to NumPy array
        dis = end_pos - start_pos
        
        # Initialize variables for each axis
        [x, y, z] = start_pos
        print(x,y,z)
        vx, vy, vz = 0, 0, 0
        ax, ay, az = 0, 0, 0

        # Initialize dictionaries to store values for all axes
        s_set, v_set, a_set = {}, {}, {}

        for t in np.arange(0, self.duration + self.dt, self.dt):
            # Acceleration phase
            if t <= self.duration / 3:
                ax = dis[0] / (2*(self.duration/3)**2)
                ay = dis[1] / (2*(self.duration/3)**2)
                az = dis[2] / (2*(self.duration/3)**2)
                
                vx = ax * t
                vy = ay * t
                vz = az * t
                
                x = x + vx * self.dt
                y = y + vy * self.dt
                z = z + vz * self.dt
            
            # Constant velocity phase
            elif t <= self.duration * 2 / 3:
                ax, ay, az = 0, 0, 0  
                x = x + vx * self.dt
                y = y + vy * self.dt
                z = z + vz * self.dt
            
            # Deceleration phase
            else:
                ax = -dis[0] / (2*(self.duration/3)**2)
                ay = -dis[1] / (2*(self.duration/3)**2)
                az = -dis[2] / (2*(self.duration/3)**2)
                
                vx = ax * (t - self.duration)
                vy = ay * (t - self.duration)
                vz = az * (t - self.duration)
                
                x = x + vx * self.dt
                y = y + vy * self.dt
                z = z + vz * self.dt
            
            # Store values in dictionaries with x, y, z keys
            s_set[t] = [ x,  y, z]
            v_set[t] = [ vx,  vy, vz]
            a_set[t] = [ ax,  ay, az]

        # # Print the results
        return t, s_set, v_set, a_set
    