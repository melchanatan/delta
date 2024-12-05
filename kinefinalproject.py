import numpy as np
import matplotlib.pyplot as plt

# Robot Parameters
f = 0.3    # Base equilateral triangle side length
e = 0.1    # End effector equilateral triangle side length
rf = 0.5   # Upper arm length
re = 0.5   # Lower arm length
tan30 = 1 / np.sqrt(3)

# P-controller constants for position and velocity control
Kp_pos = 10  # Proportional gain for position
Kp_vel = 5   # Proportional gain for velocity

# Maximum velocity and acceleration for trapezoidal profile
v_max = 0.1  # Max velocity (m/s)
a_max = 0.2  # Max acceleration (m/s^2)

# Function for Inverse Kinematics
def inverse_kinematics(x, y, z):
    def calculate_angle(x0, y0, z0):
        y1 = -0.5 * f * tan30
        y0 -= 0.5 * e * tan30

        a = (x0**2 + y0**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)
        b = (y1 - y0) / z0

        d = -(a + b * y1)**2 + rf * (b**2 * rf + rf)
        if d < 0:
            raise ValueError("Target position is not reachable.")

        yj = (y1 - a * b - np.sqrt(d)) / (b**2 + 1)
        zj = a + b * yj

        theta = np.arctan(-zj / (y1 - yj))
        return np.degrees(theta)

    theta1 = calculate_angle(x, y, z)

    cos120 = np.cos(2 * np.pi / 3)
    sin120 = np.sin(2 * np.pi / 3)
    x_prime = x * cos120 + y * sin120
    y_prime = y * cos120 - x * sin120
    theta2 = calculate_angle(x_prime, y_prime, z)

    cos240 = np.cos(4 * np.pi / 3)
    sin240 = np.sin(4 * np.pi / 3)
    x_double_prime = x * cos240 + y * sin240
    y_double_prime = y * cos240 - x * sin240
    theta3 = calculate_angle(x_double_prime, y_double_prime, z)

    return theta1, theta2, theta3

# Jacobian matrix (simplified for the delta robot)
def jacobian(theta1, theta2, theta3):
    J = np.array([
        [-rf * np.sin(np.radians(theta1)), -rf * np.sin(np.radians(theta2)), -rf * np.sin(np.radians(theta3))],
        [rf * np.cos(np.radians(theta1)), rf * np.cos(np.radians(theta2)), rf * np.cos(np.radians(theta3))],
        [0, 0, 0]
    ])
    return J

# Function for velocity control
def velocity_control(v_desired, v_current, Kp_vel):
    velocity_error = v_desired - v_current
    control_signal = Kp_vel * velocity_error
    return control_signal

# Trapezoidal trajectory generation
def generate_trapezoidal_trajectory(start, end, v_max, a_max, duration=0.25):
    # Calculate the distance and time needed for acceleration and deceleration
    distance = np.linalg.norm(end - start)
    t_acc = v_max / a_max  # time to reach max velocity
    distance_acc = 0.5 * a_max * t_acc**2  # distance covered during acceleration
    if distance_acc * 2 > distance:
        # If we can't reach max velocity (we need to accelerate and decelerate within the distance)
        t_acc = np.sqrt(distance / a_max)
        v_max = a_max * t_acc  # Adjust v_max to the available distance
        distance_acc = 0.5 * a_max * t_acc**2
    t_total = 2 * t_acc  # Total time is twice the acceleration time (acceleration + deceleration)
    
    t = np.linspace(0, duration, 100)
    trajectory = np.linspace(start, end, len(t))

    # Apply trapezoidal motion profile
    return t, trajectory

# Regularized pseudoinverse for the Jacobian matrix
def regularized_pseudoinverse(J, epsilon=1e-3):
    """Compute the regularized pseudoinverse of the Jacobian matrix."""
    try:
        return np.linalg.pinv(J + epsilon * np.eye(J.shape[0]))
    except np.linalg.LinAlgError:
        print("Singular configuration detected, skipping step.")
        return np.zeros_like(J)  # Fallback to zero matrix in case of singularity

# Function for position control
def position_control(current_pos, target_pos, Kp_pos):
    position_error = target_pos - current_pos
    v_desired = Kp_pos * position_error
    return v_desired

# Simulating the system with position control
def simulate():
    target_position = np.array([0.3, 0.2, -0.5])
    start_position = np.array([0.0, 0.0, 0.0])
    
    t, trajectory = generate_trapezoidal_trajectory(start_position, target_position, v_max, a_max)
    current_position = start_position.copy()
    v_current = np.array([0.0, 0.0, 0.0])
    
    joint_angles = []
    joint_velocities = []
    
    for i in range(len(t)):
        # Calculate desired velocity using position control
        v_desired = position_control(current_position, trajectory[i], Kp_pos)
        
        # Calculate joint angles using inverse kinematics
        x, y, z = current_position
        try:
            theta1, theta2, theta3 = inverse_kinematics(x, y, z)
        except ValueError:
            print(f"Target position at step {i} is not reachable.")
            continue
            
        joint_angles.append([theta1, theta2, theta3])
        
        # Calculate Jacobian
        J = jacobian(theta1, theta2, theta3)
        
        # Use regularized pseudoinverse to compute joint velocities
        J_pinv = regularized_pseudoinverse(J)
        joint_velocity = J_pinv.dot(v_desired)
        
        # Apply velocity control to get the controlled joint velocities
        joint_velocity_control = velocity_control(joint_velocity, v_current, Kp_vel)
        joint_velocities.append(joint_velocity_control)
        
        # Update the current position (integration step for position)
        v_current = joint_velocity_control
        current_position += v_current * (t[1] - t[0])  # simple integration to update position

    return joint_angles, joint_velocities, t

# Run the simulation
joint_angles, joint_velocities, t = simulate()

# Plot the results
plt.figure(figsize=(12, 6))

# Joint Angles Plot
plt.subplot(2, 1, 1)
plt.plot(t, np.array(joint_angles)[:, 0], label='Theta1')
plt.plot(t, np.array(joint_angles)[:, 1], label='Theta2')
plt.plot(t, np.array(joint_angles)[:, 2], label='Theta3')
plt.title('Joint Angles vs Time')
plt.xlabel('Time [s]')
plt.ylabel('Angle [degrees]')
plt.legend()

# Joint Velocities Plot
plt.subplot(2, 1, 2)
plt.plot(t, np.array(joint_velocities)[:, 0], label='Theta1 Velocity')
plt.plot(t, np.array(joint_velocities)[:, 1], label='Theta2 Velocity')
plt.plot(t, np.array(joint_velocities)[:, 2], label='Theta3 Velocity')
plt.title('Joint Velocities vs Time')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [deg/s]')
plt.legend()

plt.tight_layout()
plt.show()