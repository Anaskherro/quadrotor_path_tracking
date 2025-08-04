#!/usr/bin/env python 

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TwistStamped
from simple_pid import PID
import math

# RC Message initialization
rc_msg = OverrideRCIn()
rc_min = 1000
rc_max = 2000
roll_max = 45
pitch_max = 45


# PID controller from a library
setpoint = 4
pid = PID(10, 0, 0, setpoint)  # Setpoint is a desired speed based on the distance to target

pid.output_limits = (-pitch_max, pitch_max)  # pitch limits

current_speed = 0.0 

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
    rc_msg.channels[0] = int(mapp(roll, -roll_max, roll_max, rc_min, rc_max))
    rc_msg.channels[1] = int(mapp(pitch, -pitch_max, pitch_max, rc_min, rc_max))
    rc_msg.channels[2] = int(mapp(throttle, 0, 1, rc_min, rc_max))
    rc_msg.channels[3] = int(mapp(yaw, -1, 1, rc_min, rc_max))
    return rc_msg

def mapp(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

def speed_callback(data):
    global current_speed
    x_spd = data.twist.linear.x
    y_spd = data.twist.linear.y
    current_speed = math.sqrt(math.pow(x_spd,2)+math.pow(y_spd,2))

def pid_control(event):
    global current_speed
    pitch = pid(current_speed)
    rc_msg = rc(pitch,0,0,0.0)
    rospy.loginfo(f"Setpoint : {setpoint}  | Current speed : {current_speed} | Control : {pitch}")
    pub.publish(rc_msg)

def start():
    global pub
    arm()
    stab()
    rospy.init_node('pitch_pid', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, speed_callback)

    rospy.Timer(rospy.Duration(0.1), pid_control)  # Call pid_control at 10Hz

    rospy.spin()

if __name__ == '__main__':
    start()
