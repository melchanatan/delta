import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotParams:
    def __init__(self) -> None:
        self.f = 0.3  # Base equilateral triangle side length
        self.e = 0.1  # End effector equilateral triangle side length
        self.rf = 0.5  # Upper arm length
        self.re = 0.5  # Lower arm length
        self.tan30 = 1 / np.sqrt(3)

rp = RobotParams()

def plot_delta_robot(x, y, z, x_range=None, y_range=None, z_range=None):
    """
    Visualize the Delta Robot in 3D with upper and lower arms, angles, and custom axis ranges.
    
    Parameters:
        x, y, z : float
            Coordinates of the end-effector.
        x_range, y_range, z_range : tuple or None
            Custom ranges for the x, y, and z axes. Example: (-1, 1).
    """
    def get_triangle_vertices(side_length, z_level):
        half_side = side_length / 2
        height = rp.tan30 * side_length
        return np.array([
            [0, -height, z_level],
            [half_side, height / 2, z_level],
            [-half_side, height / 2, z_level]
        ])

    def inverse_kinematics(x, y, z):
        def calculate_angle(x0, y0, z0):
            y1 = -0.5 * rp.f * rp.tan30
            y0 -= 0.5 * rp.e * rp.tan30

            a = (x0**2 + y0**2 + z0**2 + rp.rf**2 - rp.re**2 - y1**2) / (2 * z0)
            b = (y1 - y0) / z0

            d = -(a + b * y1)**2 + rp.rf * (b**2 * rp.rf + rp.rf)
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

    try:
        theta1, theta2, theta3 = inverse_kinematics(x, y, z)
        print(f"Theta1 (Base to Upper Arm): {theta1:.2f}°")
        print(f"Theta2 (Base to Upper Arm): {theta2:.2f}°")
        print(f"Theta3 (Base to Upper Arm): {theta3:.2f}°")

        d = np.sqrt(x**2 + y**2 + z**2)
        phi1 = np.arccos((rp.rf**2 + rp.re**2 - d**2) / (2 * rp.rf * rp.re))
        phi2 = phi1
        phi3 = phi1

        print(f"Phi1 (Upper to Lower Arm): {np.degrees(phi1):.2f}°")
        print(f"Phi2 (Upper to Lower Arm): {np.degrees(phi2):.2f}°")
        print(f"Phi3 (Upper to Lower Arm): {np.degrees(phi3):.2f}°")
    except ValueError as error:
        print(error)
        return

    base_vertices = get_triangle_vertices(rp.f, 0)
    end_effector_vertices = get_triangle_vertices(rp.e, z)

    upper_arm_joints = []
    for i in range(3):
        angle = [theta1, theta2, theta3][i]
        x_joint = base_vertices[i, 0] + rp.rf * np.sin(np.radians(angle))
        z_joint = base_vertices[i, 2] - rp.rf * np.cos(np.radians(angle))
        upper_arm_joints.append([x_joint, base_vertices[i, 1], z_joint])
    upper_arm_joints = np.array(upper_arm_joints)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for i in range(len(base_vertices)):
        j = (i + 1) % len(base_vertices)
        ax.plot(
            [base_vertices[i, 0], base_vertices[j, 0]],
            [base_vertices[i, 1], base_vertices[j, 1]],
            [base_vertices[i, 2], base_vertices[j, 2]],
            'b-'
        )

    for i in range(len(end_effector_vertices)):
        j = (i + 1) % len(end_effector_vertices)
        ax.plot(
            [end_effector_vertices[i, 0], end_effector_vertices[j, 0]],
            [end_effector_vertices[i, 1], end_effector_vertices[j, 1]],
            [end_effector_vertices[i, 2], end_effector_vertices[j, 2]],
            'r-'
        )

    for i in range(len(base_vertices)):
        ax.plot(
            [base_vertices[i, 0], upper_arm_joints[i, 0]],
            [base_vertices[i, 1], upper_arm_joints[i, 1]],
            [base_vertices[i, 2], upper_arm_joints[i, 2]],
            'g-', label="Upper Arm" if i == 0 else ""
        )

    for i in range(len(upper_arm_joints)):
        ax.plot(
            [upper_arm_joints[i, 0], end_effector_vertices[i, 0]],
            [upper_arm_joints[i, 1], end_effector_vertices[i, 1]],
            [upper_arm_joints[i, 2], end_effector_vertices[i, 2]],
            'k-', label="Lower Arm" if i == 0 else ""
        )

    # Set custom axis ranges if provided
    if x_range:
        ax.set_xlim(x_range)
    if y_range:
        ax.set_ylim(y_range)
    if z_range:
        ax.set_zlim(z_range)

    # Set labels and show
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Delta Robot Visualization")
    plt.legend()
    plt.show()

# Example usage with custom ranges
plot_delta_robot(0.1, 0.1, -0.2, x_range=(-1, 1), y_range=(-1, 1), z_range=(-1, 1))

