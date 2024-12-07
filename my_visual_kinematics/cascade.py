import numpy as np
# from my_visual_kinematics.RobotDelta import RobotDelta
# from my_visual_kinematics.Frame import Frame
# from my_visual_kinematics.RobotTrajectory import RobotTrajectory
from math import pi, atan2, sqrt
class DeltaRobotController:
        # params [4, ] [r1, r2, l1, l2]
    def __init__(self, params, plot_xlim=[-0.5, 0.5], plot_ylim=[-0.5, 0.5], plot_zlim=[-1.0, 0.0],
                 ws_lim=None, ws_division=5, object_pos=[0, 0, 0], object_speed=0.2):
    # Robot.__init__(self, params, np.zeros([3, ]), plot_xlim, plot_ylim, plot_zlim, ws_lim, ws_division)
        self.is_reachable_forward = True
        self.moving_object_pos = [object_pos[0]], [object_pos[1]], [object_pos[2]]
        self.object_speed = object_speed

    # ================== Definition of r1, r2, l1, l2
    #                  r1
    #              C ========== -------
    #            l1 //   O    \\ / theta
    #              //          \\
    #            B \\---D      //
    #            l2 \\        //
    #                \\  P   //
    #                A ======
    #                  r2
    # ==================
    def move_object(self, x):
        self.moving_object_pos = [[x], self.moving_object_pos[1], self.moving_object_pos[2]]  # Update robot's position

    @property
    def ob_x(self):
        return self.moving_object_pos[0]
    
    @property
    def r1(self):
        return self.params[0]

    @property
    def r2(self):
        return self.params[1]

    @property
    def l1(self):
        return self.params[2]

    @property
    def l2(self):
        return self.params[3]

    # ================== Definition of phi [3, ]
    #                  \
    #                   \  phi[1] = 2/3*pi
    #                    \
    #                      -----------  phi[0] = 0
    #                    /
    #   phi[2] = 4/3*pi /
    #                  /
    # ==================
    @property
    def phi(self):
        return np.array([0., 2.*pi/3., 4.*pi/3.])

    # ================== Definition of oc [3, 3]
    # |  oc1_x  |  oc2_x  |  oc3_x  |
    # |  oc1_y  |  oc2_y  |  oc3_y  |
    # |  oc1_z  |  oc2_z  |  oc3_z  |
    # ==================
    @property
    def oc(self):
        oc = np.zeros([3, 3], dtype=np.float64)
        oc[0] = np.cos(self.phi) * self.r1
        oc[1] = np.sin(self.phi) * self.r1
        return oc

    # ================== Definition of cb [3, 3]
    # |  cb1_x  |  cb2_x  |  cb3_x  |
    # |  cb1_y  |  cb2_y  |  cb3_y  |
    # |  cb1_z  |  cb2_z  |  cb3_z  |
    # ==================
    @property
    def cb(self):
        cb = np.zeros([3, 3], dtype=np.float64)
        cb[0] = np.cos(self.phi) * np.cos(self.axis_values) * self.l1
        cb[1] = np.sin(self.phi) * np.cos(self.axis_values) * self.l1
        cb[2] = - np.sin(self.axis_values) * self.l1
        return cb

    # ================== Definition of ap [3, 3]
    # |  ap1_x  |  ap2_x  |  ap3_x  |
    # |  ap1_y  |  ap2_y  |  ap3_y  |
    # |  ap1_z  |  ap2_z  |  ap3_z  |
    # ==================
    @property
    def ap(self):
        ap = np.zeros([3, 3], dtype=np.float64)
        ap[0] = - np.cos(self.phi) * self.r2
        ap[1] = - np.sin(self.phi) * self.r2
        return ap

    # ================== bd = ap [3 ,3]
    @property
    def bd(self):
        return self.ap

    # ================== od [3 ,3]
    @property
    def od(self):
        return self.oc + self.cb + self.bd

    # ================== op (is_reachable, [3, 1])
    @property
    def op(self):
        # solve for circle centroid
        od = self.od
        temp_p = np.ones([4, 4], dtype=np.float64)
        temp_p[1:4, 0:3] = od.T
        temp_a = np.zeros([3, 3])
        temp_y = np.zeros([3, 1])

        temp_a[0, 0] = np.linalg.det(np.delete(np.delete(temp_p, 0, axis=0), 0, axis=1))
        temp_a[0, 1] = - np.linalg.det(np.delete(np.delete(temp_p, 0, axis=0), 1, axis=1))
        temp_a[0, 2] = np.linalg.det(np.delete(np.delete(temp_p, 0, axis=0), 2, axis=1))
        temp_y[0, 0] = - np.linalg.det(od)

        temp_a[1, 0] = 2 * (od[0, 1] - od[0, 0])
        temp_a[1, 1] = 2 * (od[1, 1] - od[1, 0])
        temp_a[1, 2] = 2 * (od[2, 1] - od[2, 0])
        temp_y[1, 0] = np.linalg.norm(od[:, 0]) ** 2 - np.linalg.norm(od[:, 1]) ** 2

        temp_a[2, 0] = 2 * (od[0, 2] - od[0, 0])
        temp_a[2, 1] = 2 * (od[1, 2] - od[1, 0])
        temp_a[2, 2] = 2 * (od[2, 2] - od[2, 0])
        temp_y[2, 0] = np.linalg.norm(od[:, 0]) ** 2 - np.linalg.norm(od[:, 2]) ** 2

        oe = - np.linalg.inv(temp_a).dot(temp_y)
        r = np.linalg.norm(oe - od[:, 0:1])
        if r > self.l2:
            logging.error("Pose cannot be reached narak!")
            self.is_reachable_inverse = False
            return oe  # False : not reachable
        else:
            vec = np.cross(od[:, 2]-od[:, 1], od[:, 2]-od[:, 0])
            vec = vec / np.linalg.norm(vec) * np.sqrt(self.l2*self.l2 - r*r)
            op = oe + vec.reshape([3, 1])
            self.is_reachable_inverse = True
            return op  # True : reachable

    @property
    def end_frame(self):
        return Frame.from_r_3_3(np.eye(3, dtype=np.float64), self.op)

    def inverse(self, end):
        op = end.t_3_1.flatten()
        # solve a*sinx + b*cosx = c
        a = 2 * self.l1 * op[2]
        theta = np.zeros([3, ], dtype=np.float64)
        self.is_reachable_inverse = True
        for i in range(3):
            oa = op - self.ap[:, i]
            b = 2 * self.l1 * (np.cos(self.phi[i]) * (self.r1 * np.cos(self.phi[i]) - oa[0])
                               + np.sin(self.phi[i]) * (self.r1 * np.sin(self.phi[i]) - oa[1]))
            c = self.l2 * self.l2 - self.l1 * self.l1 - np.linalg.norm(oa) ** 2 \
                - self.r1 * self.r1 + 2 * self.r1 * (np.cos(self.phi[i]) * oa[0] + np.sin(self.phi[i]) * oa[1])
            if a*a + b*b > c*c:
                theta[i] = simplify_angle(atan2(c, -sqrt(a*a+b*b-c*c)) - atan2(b, a))
            else:
                self.is_reachable_inverse = False
                break
        if not self.is_reachable_inverse:
            logging.error("Pose cannot be reached mark!")
            return None
        self.forward(theta)
        return theta
    
    def inverse_kinematics_with_velocity(self, x,y,z,vx,vy,vz):
        theta1, theta2, theta3 = self.inverse(x,y,z)
        J = self.jacobian(theta1, theta2, theta3)
        cartesian_velocity = np.array(vx,vy,vz)
        joint_velocity = np.linalg.pinv(J).dot(cartesian_velocity)
        return (theta1, theta2, theta3), joint_velocity

    def jacobian(self, theta1, theta2, theta3):
        return np.array([
            [-self.rf * np.sin(np.radians(theta1)), -self.rf * np.sin(np.radians(theta2)), -self.rf * np.sin(np.radians(theta3))],
            [self.rf * np.cos(np.radians(theta1)), self.rf * np.cos(np.radians(theta2)), self.rf * np.cos(np.radians(theta3))],
            [0, 0, 0]
        ])

    

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
        
        return t, trajectory, velocities

    

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
