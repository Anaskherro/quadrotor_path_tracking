#!/usr/bin/env python
import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped , TwistStamped
import math
from simple_pid import PID

# RC Message initialization
rc_msg = OverrideRCIn()
rc_min = 1000
rc_max = 2000
roll_max = 45
pitch_max = 45

# PID controller from a library
setpoint = 10
pid = PID(0.33, 0.3, 0.15, setpoint)  # Setpoint is target_z
pid.output_limits = (0, 1)  # Throttle limits

current_z = 0.0  # Initialize current_z
setpoint_spd = 2
pid2 = PID(15, 1, 0, setpoint_spd)  # Setpoint is a desired speed based on the distance to target

pid2.output_limits = (-pitch_max, pitch_max)  # pitch limits

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

def pose_callback(data):
    global current_z
    current_z = data.pose.position.z

def speed_callback(data):
    global current_speed
    x_spd = data.twist.linear.x
    y_spd = data.twist.linear.y
    current_speed = math.sqrt(math.pow(x_spd,2)+math.pow(y_spd,2))



def pid_control(event):
    global current_z, current_speed
    throttle = pid(current_z)
    pitch = pid2(current_speed)

    rc_msg = rc(pitch, 0, 0, throttle)
    rospy.loginfo(f"Setpoint >  ALT : {setpoint} | SPD : {setpoint_spd}  | Current Alt : {current_z} | current Spd :{current_speed} | Pitch : {pitch}")
    pub.publish(rc_msg)

def start():
    global pub
    arm()
    stab()
    rospy.init_node('alt_pid', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, speed_callback)
    # Use a timer to periodically call the pid_control function
    rospy.Timer(rospy.Duration(0.05), pid_control)  # Call pid_control at 10Hz

    rospy.spin()

if __name__ == '__main__':
    start()
