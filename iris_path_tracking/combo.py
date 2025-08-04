#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TwistStamped, Pose2D
from simple_pid import PID
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
from tf.transformations import euler_from_quaternion
import math

# RC Message initialization
rc_msg = OverrideRCIn()
rc_min = 1000
rc_max = 2000
roll_max = 45
pitch_max = 45


# Altitude PID 
alt_setpoint = 10
alt_pid = PID(0.33, 0.3, 0.15, alt_setpoint)  # Setpoint is target_z
alt_pid.output_limits = (0, 1)  # Throttle limits
current_alt = 0.0

# Yaw PID 
yaw_setpoint = 2# Target yaw in rad
yaw_pid = PID(0.3, 0.006,0.08, yaw_setpoint)
yaw_pid.output_limits = (-1, 1)  # Yaw control limits in the range of -1 to 1
current_yaw = 0.0

# Speed PID 
spd_setpoint = 1
spd_pid = PID(10, 0, 0, spd_setpoint)  # Setpoint is a desired speed based on the distance to target
spd_pid.output_limits = (-10, 10)  # pitch limits

# # Pitch pid 
# k1 = 1
# k2 = 0.5

# current_pitch = 0
# current_pitch_rate = 0 
# rate_p = PID(k1,0,0)
# ang_p = PID(k2,0,0)
current_pitch = 0.0
current_speed = 0.0 
euler_distance = 0
x_set = 0
y_set = 0
bearing = 0.0
pitch = 0.0
pitch_set = 0.0
yaw_output = 0.0



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

def get_bearing(x1,y1,x2,y2):

    delta_x = x2 - x1
    delta_y = y2 - y1
    bearing_rad = math.atan2(delta_y, delta_x)
    return bearing_rad

def setpoint_callback(data):
    global x_set, y_set, pitch_set  
    x_set = data.x
    y_set = data.y
    # pitch_set = data.theta
    rospy.loginfo(f"Received setpoint: x={data.x}, y={data.y}")

def alt_callback(data):
    global current_alt,current_x, current_y, bearing, euler_distance, x_set,y_set
    current_alt = data.pose.pose.position.z
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y   
    euler_distance = math.sqrt(math.pow(x_set - current_x,2)+math.pow(y_set - current_y,2))
    bearing = get_bearing(x_set,y_set,current_x,current_y)
  

def yaw_callback(data):
    global current_yaw, current_pitch
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw= yaw  # Yaw in radians
    current_pitch = math.degrees(pitch)

def speed_callback(data):
    global current_speed
    x_spd = data.twist.linear.x
    y_spd = data.twist.linear.y
    current_speed = math.sqrt(math.pow(x_spd,2)+math.pow(y_spd,2))

def pitch_rate_callback(data):
    global current_pitch_rate
    current_pitch_rate = data.angular_velocity.x

def wrap_360(angle):
    res = angle % 360
    if res < 0:
        res += 360
    return res

def angle_diff(a, b):
    dif = wrap_360(b - a + 180)
    return dif - 180

def normalize_angle(angle):
    return angle % 360

def pid_control(event):

    global bearing , pitch, yaw_output
    yaw_pid.setpoint = bearing

    # rate_p.setpoint = pitch_set
    # read_cmd = rate_p(current_pitch)
    # ang_p.setpoint= read_cmd
    # pitch_cmd = ang_p(current_pitch_rate)
    
    if(abs(current_yaw - bearing) <= 0.05): 
        pitch = spd_pid(current_speed)
        feed_forward_output = 0.1 * pitch 
        alt_pid.setpoint=alt_setpoint+feed_forward_output

    if euler_distance <= 1: spd_pid.setpoint = 0
    if(abs(alt_setpoint -current_alt) <= 0.3 and current_speed <= 0.1) : yaw_output = yaw_pid(current_yaw)

    throttle = alt_pid(current_alt)
    

    
    angle_deg = math.degrees(current_yaw)
    br = math.degrees(bearing)
    diff = normalize_angle(angle_diff(angle_deg, br))

    if abs(diff) > 5:  yaw_output = math.copysign(yaw_output, diff)
    else: yaw_output = 0  

    rc_msg = rc(pitch, 0, yaw_output, throttle)
    rospy.loginfo(f"altitude: {current_alt} -- {alt_setpoint}   || yaw : {current_yaw} -- {bearing} -- control : {yaw_output} || pitch {current_pitch} -- {pitch} dist : {euler_distance}")
    pub.publish(rc_msg)

def start():
    global pub, current_yaw_pub, desired_yaw_pub

    arm()
    stab()

    rospy.init_node('combo_pid', anonymous=True)

    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.Subscriber('/mavros/global_position/local', Odometry, alt_callback)
    rospy.Subscriber('/mavros/global_position/local', Odometry, yaw_callback)
    rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, speed_callback)
    rospy.Subscriber('/mavros/imu/data', Imu, pitch_rate_callback)
    rospy.Subscriber('/setpoint', Pose2D, setpoint_callback)

    rospy.Timer(rospy.Duration(0.01), pid_control)  

    rospy.spin()

if __name__ == '__main__':

    start()