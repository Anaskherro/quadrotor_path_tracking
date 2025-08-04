import rospy

from mav_msgs.msg import Actuators

import math
def callback(data):
    global Omega
    o1 = data.angular_velocities[0]
    o2 = data.angular_velocities[1]
    o3 = data.angular_velocities[2]
    o4 = data.angular_velocities[3]
    Omega = (o1 + o2 + o3 + o4)/4
    print(Omega)

def listener():
    rospy.init_node('listener', anonymous=True)  # Initialize the node with a unique name
    rospy.Subscriber('/hummingbird/gazebo/command/motor_speed', Actuators, callback)
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    listener()
