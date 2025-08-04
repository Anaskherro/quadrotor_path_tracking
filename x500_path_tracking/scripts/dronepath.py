#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

# Global variable to store the recorded path
recorded_path = []
path_pub = None  # Publisher for the traversed path

traversed_path = Path()

def pose_callback(odom):
    global recorded_path, traversed_path
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    recorded_path.append((x, y))
    pose_stamped = PoseStamped()
    pose_stamped.header = odom.header
    pose_stamped.pose.position = odom.pose.pose.position
    traversed_path.poses.append(pose_stamped)
    traversed_path.header.frame_id = "uav1/gps_baro_origin"  # Set frame ID
    path_pub.publish(traversed_path)
def main():
    global path_pub

    # Initializing ROS node.
    rospy.init_node("path_recorder_and_publisher", anonymous=True)

    # Create a subscriber for local position odom
    rospy.Subscriber('/uav1/estimation_manager/odom_main', Odometry, pose_callback)

    # Create a publisher for the traversed path
    path_pub = rospy.Publisher('traversed_path', Path, queue_size=100)

    # print("Recording path...")

    # Spin until Ctrl+C is pressed
    rospy.spin()

    # # Write recorded path to file
    # with open('record_x500.txt', 'w') as f:
    #     for point in recorded_path:
    #         f.write(f"{point[0]}, {point[1]}\n")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
