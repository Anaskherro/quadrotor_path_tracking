import math

def calculate_yaw(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

def waypoints_with_yaw(waypoints):
    waypoints_yaw = []

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        yaw = calculate_yaw(x1, y1, x2, y2)
        waypoints_yaw.append((x1, y1, yaw))

    # For the last waypoint, repeat the last calculated yaw
    if len(waypoints) > 1:
        last_yaw = calculate_yaw(waypoints[-2][0], waypoints[-2][1], waypoints[-1][0], waypoints[-1][1])
        waypoints_yaw.append((waypoints[-1][0], waypoints[-1][1], last_yaw))
    
    return waypoints_yaw

def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split(','))
            waypoints.append((x, y))
    return waypoints

def write_waypoints_with_yaw(file_path, waypoints_yaw):
    with open(file_path, 'w') as file:
        for wp in waypoints_yaw:
            file.write(f"{wp[0]},{wp[1]},{wp[2]}\n")

# Paths to the input and output files
input_file = 'path2.txt'
output_file = 'path2yaw.txt'

# Read waypoints from input file
waypoints = read_waypoints(input_file)

# Calculate waypoints with yaw
waypoints_yaw = waypoints_with_yaw(waypoints)

# Write waypoints with yaw to output file
write_waypoints_with_yaw(output_file, waypoints_yaw)

print(f"Waypoints with yaw have been written to {output_file}")
