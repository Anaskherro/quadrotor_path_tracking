import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Calculate yaw, pitch, and roll
def calculate_yaw(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

def calculate_pitch(z1, z2, x1, x2):
    return math.atan2(z2 - z1, x2 - x1)

def calculate_roll(z1, z2, y1, y2):
    return math.atan2(z2 - z1, y2 - y1)

def waypoints_with_yaw_pitch_roll(waypoints, z):
    waypoints_ypr = []

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        yaw = calculate_yaw(x1, y1, x2, y2)
        pitch = calculate_pitch(z, z, x1, x2)  # Z is constant
        roll = calculate_roll(z, z, y1, y2)    # Z is constant
        waypoints_ypr.append((x1, y1, z, yaw, pitch, roll))

    # For the last waypoint, repeat the last calculated yaw, pitch, and roll
    if len(waypoints) > 1:
        last_yaw = calculate_yaw(waypoints[-2][0], waypoints[-2][1], waypoints[-1][0], waypoints[-1][1])
        last_pitch = calculate_pitch(z, z, waypoints[-2][0], waypoints[-1][0])
        last_roll = calculate_roll(z, z, waypoints[-2][1], waypoints[-1][1])
        waypoints_ypr.append((waypoints[-1][0], waypoints[-1][1], z, last_yaw, last_pitch, last_roll))
    
    return waypoints_ypr

def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split(','))
            waypoints.append((x, y))
    return waypoints

def write_waypoints_with_ypr(file_path, waypoints_ypr):
    with open(file_path, 'w') as file:
        for wp in waypoints_ypr:
            file.write(f"{wp[0]},{wp[1]},{wp[2]},{wp[3]},{wp[4]},{wp[5]}\n")

def plot_waypoints_with_ypr(waypoints_ypr):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x_coords = [wp[0] for wp in waypoints_ypr]
    y_coords = [wp[1] for wp in waypoints_ypr]
    z_coords = [wp[2] for wp in waypoints_ypr]

    ax.plot(x_coords, y_coords, z_coords, 'bo-', label='Waypoints')

    def plot_circle(ax, x, y, z, angle, radius=0.5, color='r'):
        theta = np.linspace(0, 2*np.pi, 100)
        x_circle = x + radius * np.cos(theta) * np.cos(angle)
        y_circle = y + radius * np.sin(theta) * np.cos(angle)
        z_circle = z + radius * np.sin(angle)
        ax.plot(x_circle, y_circle, z_circle, color=color)

    for (x, y, z, yaw, pitch, roll) in waypoints_ypr:
        plot_circle(ax, x, y, z, yaw, color='r')
        plot_circle(ax, x, y, z, pitch, color='g')
        plot_circle(ax, x, y, z, roll, color='b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Waypoints with Yaw, Pitch, and Roll')
    ax.legend(['Yaw', 'Pitch', 'Roll'])
    plt.show()

# Paths to the input and output files
input_file = 'path2.txt'
output_file = 'path2ypr.txt'

# Read waypoints from input file
waypoints = read_waypoints(input_file)

# Define a constant height
z_height = 5

# Calculate waypoints with yaw, pitch, and roll
waypoints_ypr = waypoints_with_yaw_pitch_roll(waypoints, z_height)

# Write waypoints with yaw, pitch, and roll to output file
write_waypoints_with_ypr(output_file, waypoints_ypr)

# Plot the waypoints with yaw, pitch, and roll
plot_waypoints_with_ypr(waypoints_ypr)

print(f"Waypoints with yaw, pitch, and roll have been written to {output_file}")
