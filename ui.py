import pygame
import sys
import csv
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

    # Initialize WorkspaceValidator
    workspace_validator = WorkspaceValidator(
        robot_controller.f,
        robot_controller.e,
        robot_controller.rf,
        robot_controller.re,
        robot_controller.tan30
    )
    
    # Initial position
    x, y, z = x_old, y_old, z_old = -0.4, -0.2, -0.5


# Initialize PyGame
pygame.init()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
BLUE = (50, 100, 255)
PINK = (255, 182, 193)  # Pink for hover
BUTTON_DEFAULT = (240, 240, 240)  # Default button color
BUTTON_BORDER = (0, 0, 0)  # Button border color
FONT = pygame.font.SysFont("arial", 24)
HEADER_FONT = pygame.font.SysFont("arial", 32, bold=True)


class InputBox:
    def __init__(self, x, y, w, h, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.color_inactive = GRAY
        self.color_active = BLUE
        self.color = self.color_inactive
        self.text = ""
        self.active = False
        self.label = label  # Label for the input (e.g., "x", "y", "z")

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            # Toggle active state if clicked
            self.active = self.rect.collidepoint(event.pos)
            self.color = self.color_active if self.active else self.color_inactive

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                self.active = False  # Deactivate on "Enter"
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                self.text += event.unicode

    def draw(self, surface):
        # Draw the input box and its label
        label_surface = FONT.render(f"{self.label}:", True, BLACK)
        surface.blit(label_surface, (self.rect.x - 50, self.rect.y + 5))
        pygame.draw.rect(surface, self.color, self.rect, 2)
        text_surface = FONT.render(self.text, True, BLACK)
        surface.blit(text_surface, (self.rect.x + 5, self.rect.y + 5))


class Button:
    def __init__(self, x, y, w, h, text, callback=None):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.callback = callback  # Function to execute on click
        self.hover = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos) and self.callback:
                self.callback()

    def draw(self, surface):
        # Button color changes based on hover state
        color = BUTTON_DEFAULT
        if self.hover:
            color = PINK
        pygame.draw.rect(surface, color, self.rect)
        pygame.draw.rect(surface, BUTTON_BORDER, self.rect, 2)

        # Draw button text
        text_surface = FONT.render(self.text, True, BLACK)
        surface.blit(text_surface, (self.rect.x + (self.rect.width - text_surface.get_width()) // 2,
                                    self.rect.y + (self.rect.height - text_surface.get_height()) // 2))

    def update_hover_state(self, mouse_pos):
        self.hover = self.rect.collidepoint(mouse_pos)


class App:
    def __init__(self):
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("3 DOF Delta Robot")
        self.clock = pygame.time.Clock()
        self.running = True

        # Input boxes for start and target positions with default values set to "0.0"
        self.start_input_boxes = [
            InputBox(500, 150, 140, 32, "x"),
            InputBox(500, 200, 140, 32, "y"),
            InputBox(500, 250, 140, 32, "z"),
        ]
        self.target_input_boxes = [
            InputBox(500, 350, 140, 32, "x"),
            InputBox(500, 400, 140, 32, "y"),
            InputBox(500, 450, 140, 32, "z"),
        ]

        # Set default values
        for box in self.start_input_boxes + self.target_input_boxes:
            box.text = "0.0"

        # Create "Download CSV" Button
        self.download_button = Button(350, 500, 300, 50, "Calculate & Get CSV", self.download_csv)

    def download_csv(self):
        """Callback for the 'Download CSV' button."""
        # Get input values for start and target positions
        start_values = [box.text for box in self.start_input_boxes]
        target_values = [box.text for box in self.target_input_boxes]

        # Save to a CSV file
        with open("robot_data.csv", mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Type", "x", "y", "z"])
            writer.writerow(["Start"] + start_values)
            writer.writerow(["Target"] + target_values)

        print("CSV file downloaded: robot_data.csv")

    def handle_events(self):
        mouse_pos = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            # Delegate events to input boxes and button
            for input_box in self.start_input_boxes + self.target_input_boxes:
                input_box.handle_event(event)
            self.download_button.handle_event(event)

        # Update hover state for the button
        self.download_button.update_hover_state(mouse_pos)

    def draw(self):
        """Render everything on the screen."""
        self.screen.fill(WHITE)

        # Draw title using smaller font
        small_header_font = pygame.font.SysFont("arial", 24, bold=True)
        title_surface = small_header_font.render("3 DOF Delta Robot", True, BLACK)
        self.screen.blit(title_surface, (300, 30))  # Adjusted position for smaller font

        # Draw section labels
        start_label = FONT.render("Start Position:", True, BLACK)
        target_label = FONT.render("Target Position:", True, BLACK)
        self.screen.blit(start_label, (200, 150))
        self.screen.blit(target_label, (200, 400))

        # Draw input boxes
        for input_box in self.start_input_boxes + self.target_input_boxes:
            input_box.draw(self.screen)

        # Draw button
        self.download_button.draw(self.screen)

        # Draw robot dimensions and load details
        dimension_label = small_header_font.render("Robot Dimensions and Load", True, BLACK)
        self.screen.blit(dimension_label, (50, 60))  # Adjusted position for smaller header
        dimensions = [
            f"Base Triangle Side Length (f): {robot_controller.f} m",
            f"End Effector Triangle Side Length (e): {robot_controller.e} m",
            f"Upper Arm Length (rf): {robot_controller.rf} m",
            f"Lower Arm Length (re): {robot_controller.re} m",
            f"Assumed Weight: 0.0 kg",
            f"Load: 0.5 kg",
        ]

        # Use a smaller font for the dimensions
        small_font = pygame.font.SysFont("arial", 14)  # Smaller font size
        for i, line in enumerate(dimensions):
            dimension_surface = small_font.render(line, True, BLACK)
            self.screen.blit(dimension_surface, (50, 100 + i * 20))  # Adjusted spacing for smaller text

        # Update the display
        pygame.display.flip()

    def run(self):
        """Run the main loop."""
        while self.running:
            self.handle_events()
            self.draw()
            self.clock.tick(30)





if __name__ == "__main__":
    app = App()
    app.run()
