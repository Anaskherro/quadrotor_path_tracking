#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros

def publish_static_transform():
    rospy.init_node('static_transform_publisher', anonymous=True)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "uav1/world_origin"
    static_transformStamped.child_frame_id = "map"
    
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        static_transformStamped.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(static_transformStamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_static_transform()
    except rospy.ROSInterruptException:
        pass
