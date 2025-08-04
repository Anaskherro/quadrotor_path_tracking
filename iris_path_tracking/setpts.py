#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def callback(data):
    rospy.loginfo(f"Received Pose2D: x={data.x}, y={data.y}, theta={data.theta}")

def pose2d_node():
    rospy.init_node('setpoint_node', anonymous=True)

    pub = rospy.Publisher('setpoint', Pose2D, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            input_str = input("Enter setpoints (x,y,pitch): ")
            x_str, y_str, theta_str= input_str.split(',')
            x = float(x_str.strip())
            y = float(y_str.strip())
            theta = float(theta_str.strip())
            # theta = float(theta_str.strip())

            pose2d = Pose2D()
            pose2d.x = x
            pose2d.y = y
            pose2d.theta = theta

            rospy.loginfo(f"Publishing Pose2D: x={x}, y={y}, pitch={theta}")
            pub.publish(pose2d)

            rate.sleep()
        except ValueError:
            rospy.logwarn("Invalid input. Please enter the setpoints in the format: x,y")
        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
    try:
        pose2d_node()
    except rospy.ROSInterruptException:
        pass
