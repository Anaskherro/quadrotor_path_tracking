import matplotlib.pyplot as plt
import numpy as np

def read_waypoints_with_yaw(file_path):
    waypoints_yaw = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y, yaw = map(float, line.strip().split(','))
            waypoints_yaw.append((x, y, yaw))
    return waypoints_yaw

def plot_waypoints_with_yaw(waypoints_yaw):
    x_coords = [wp[0] for wp in waypoints_yaw]
    y_coords = [wp[1] for wp in waypoints_yaw]
    yaws = [wp[2] for wp in waypoints_yaw]

    plt.figure()
    plt.plot(x_coords, y_coords, 'bo-', label='Waypoints')

    for (x, y, yaw) in waypoints_yaw:
        dx = np.cos(yaw)
        dy = np.sin(yaw)
        plt.arrow(x, y, dx, dy, head_width=0.1, head_length=0.2,overhang=0,fc='r', ec='r')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Waypoints with Yaw')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()

# Path to the input file
input_file = 'path2yaw.txt'

# Read waypoints with yaw from the input file
waypoints_yaw = read_waypoints_with_yaw(input_file)

# Plot the waypoints with yaw
plot_waypoints_with_yaw(waypoints_yaw)
