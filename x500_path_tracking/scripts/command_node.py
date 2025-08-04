#!/usr/bin/env python

import rospy
import math
from std_srvs.srv import Empty
from mrs_msgs.srv import Vec4, Vec4Request

def calculate_heading_change(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dy, dx)

def read_trajectory_from_file(file_path, angle_threshold):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        trajectory = []
        prev_point = None
        for line in lines:
            x, y, _ = map(float, line.strip().split(','))
            if prev_point is not None:
                if len(trajectory) > 0:
                    prev_heading = calculate_heading_change(trajectory[-1], prev_point)
                    current_heading = calculate_heading_change(prev_point, (x, y))
                    heading_change = abs(current_heading - prev_heading)
                    if heading_change >= angle_threshold:
                        trajectory.append((prev_point[0], prev_point[1], 3, prev_heading))  # Add the corner point
            prev_point = (x, y)
        if prev_point is not None:
            if len(trajectory) > 0:
                trajectory.append((*prev_point, 3, calculate_heading_change(trajectory[-1], prev_point)))
            else:
                trajectory.append((*prev_point, 3, 0.0))  # If the trajectory is empty, set heading to 0.0
    return trajectory


def main():
    rospy.init_node("publish_trajectory", anonymous=True)

    trajectory_file = 'path2heading.txt'
    trajectory = read_trajectory_from_file(trajectory_file, angle_threshold=0.1)  # Adjust angle_threshold as needed

    # Call takeoff service
    rospy.wait_for_service('/uav1/uav_manager/takeoff')
    try:
        takeoff = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Empty)
        takeoff()
        rospy.loginfo("Takeoff command sent")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    # Call goto service for each point in the trajectory
    rospy.wait_for_service('/uav1/control_manager/goto')
    for point in trajectory:
        x, y, _, heading = point
        try:
            goto = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
            goto_request = Vec4Request()
            goto_request.x = x
            goto_request.y = y
            goto_request.z = 1.5
            goto_request.heading = heading
            goto(goto_request)
            rospy.loginfo(f"Go to point ({x}, {y}) command sent")
            rospy.sleep(1)  # Adjust as needed
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    # Call land service
    rospy.wait_for_service('/uav1/uav_manager/land')
    try:
        land = rospy.ServiceProxy('/uav1/uav_manager/land', Empty)
        land()
        rospy.loginfo("Land command sent")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
