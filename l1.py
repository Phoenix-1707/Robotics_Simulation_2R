import math
import numpy as np
import matplotlib.pyplot as plt

# Define lengths of the two links of the robotic arm
L1 = 1
L2 = 2

# Function to calculate the end effector position for given joint angles
def calculate_end_effector(theta1, theta2):
    # Calculate the position of the end effector based on joint angles
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return x2, y2

# Function to calculate inverse kinematics
def inverse_kinematics(x_end, y_end):
    # Calculate the joint angles for a given end effector position
    D = (x_end ** 2 + y_end ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    theta2 = math.acos(D)
    theta1 = math.atan2(y_end, x_end) - math.atan2((L2 * np.sin(theta2)), (L1 + L2 * np.cos(theta2)))
    return theta1, theta2

# Function to move the end effector to the specified coordinates
def move_to_coordinates(x_end, y_end):
    # Check if the desired end effector position is within the workspace
    if x_end < -(L1 + L2) or x_end > (L1 + L2) or y_end < -(L1 + L2) or y_end > (L1 + L2):
        print("Error: Desired end effector position is out of workspace.")
        return

    # Calculate inverse kinematics
    theta1_start = 0
    theta2_start = 0
    theta1_end, theta2_end = inverse_kinematics(x_end, y_end)

    # Print the calculated inverse angles
    print("Inverse angles (theta1, theta2):", math.degrees(theta1_end), math.degrees(theta2_end))

    # Move the end effector to the specified coordinates
    num_frames = 50
    for i in range(num_frames):
        theta1 = theta1_start + i * (theta1_end - theta1_start) / num_frames
        theta2 = theta2_start + i * (theta2_end - theta2_start) / num_frames
        x, y = calculate_end_effector(theta1, theta2)
        print("Current position of end effector (x, y):", x, y)
        plt.plot([0, L1*np.cos(theta1), x], [0, L1*np.sin(theta1), y], 'r-o')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2R Robotic Arm')
        plt.grid(True)  # Display a static grid
        plt.pause(0.1)  # Pause for 0.1 seconds to display each frame
    plt.show()

# Accept end effector coordinates from the terminal
x_end = float(input("Enter the x-coordinate of the end effector: "))
y_end = float(input("Enter the y-coordinate of the end effector: "))

# Move the end effector to the specified coordinates
try:
    move_to_coordinates(x_end, y_end)
except ValueError as e:
    print("Error:", e)
