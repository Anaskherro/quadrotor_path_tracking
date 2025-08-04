#!/usr/bin/env python

import rospy
import math
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from simple_pid import PID

# RC Message initialization
rc_msg = OverrideRCIn()
rc_min = 1000
rc_max = 2000
roll_max = 45
pitch_max = 45

# PID controller initialization
setpoint = 3 # Target yaw in rad
pid = PID(0.16, 0.007,0.0, setpoint)
pid.output_limits = (-1, 1)  # Yaw control limits in the range of -1 to 1
 # Initialize current_yaw
current_yaw = 0.0
def arm():
    print("\nArming")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value=True)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Arming failed: %s" % e)

def stab():
    print("\nSetting Mode")
    rospy.wait_for_service('/mavros/set_mode')
    try:
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = change_mode(custom_mode="STABILIZE")
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Set mode failed: %s" % e)

def rc(pitch, roll, yaw, throttle):
    # rc_msg.channels[0] = int(mapp(roll, -roll_max, roll_max, rc_min, rc_max))
    # rc_msg.channels[1] = int(mapp(pitch, -pitch_max, pitch_max, rc_min, rc_max))
    rc_msg.channels[2] = int(mapp(throttle, 0, 1, rc_min, rc_max))
    rc_msg.channels[3] = int(mapp(yaw, -1, 1, rc_min, rc_max))
    return rc_msg

def mapp(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

def yaw_callback(data):
    global current_yaw
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw= yaw  # Yaw in radians

def pid_control(event):

    yaw_output = pid(current_yaw)
    rc_msg = rc(0, 0, yaw_output, 0.6)
    rospy.loginfo(f"Setpoint: {setpoint} | Current yaw (deg): {current_yaw} | Control: {yaw_output}")
    pub.publish(rc_msg)

def start():
    global pub
    arm()
    stab()
    rospy.init_node('yaw_pid', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.Subscriber('/mavros/global_position/local', Odometry, yaw_callback)

    # Use a timer to periodically call the pid_control function
    rospy.Timer(rospy.Duration(0.1), pid_control) 

    rospy.spin()

if __name__ == '__main__':
    start()
