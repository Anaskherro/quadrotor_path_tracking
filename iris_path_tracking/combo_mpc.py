#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import do_mpc
from casadi import *

# RC Message initialization
rc_msg = OverrideRCIn()
rc_min = 1000
rc_max = 2000
roll_max = 500
pitch_max = 500

# Setpoints
alt_setpoint = 10
x_set = 0
y_set = 0

# Current state
current_alt = 0.0
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
current_speed = 0.0
euler_distance = 0.0
bearing = 0.0
current_vx=0.0
current_vy=0.0
current_vz=0.0
current_roll=0.0
current_pitch=0.0
current_p = 0.0
current_q = 0.0
current_r = 0.0
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
        response = change_mode(custom_mode="Alt_Hold")
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Set mode failed: %s" % e)

def rc(pitch, roll, yaw, throttle):
    rc_msg.channels[0] = int(mapp(roll, -roll_max, roll_max, rc_min, rc_max))
    rc_msg.channels[1] = int(mapp(pitch, -pitch_max, pitch_max, rc_min, rc_max))
    rc_msg.channels[2] = int(mapp(throttle, 0, 1, rc_min, rc_max))
    rc_msg.channels[3] = int(mapp(yaw, -500, 500, rc_min, rc_max))
    return rc_msg

def mapp(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin
    
def alt_callback(data):
    global current_alt, current_x, current_y, euler_distance, bearing
    current_alt = data.pose.pose.position.z
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    euler_distance = math.sqrt(math.pow(x_set - current_x, 2) + math.pow(y_set - current_y, 2))
    bearing = get_bearing(x_set, y_set, current_x, current_y)


def yaw_callback(data):
    global current_yaw,current_pitch, current_roll
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw  # Yaw in radians
    current_roll = roll
    current_pitch = pitch 

def speed_callback(data):
    global current_speed,current_vx, current_vy, current_vz
    x_spd = data.twist.linear.x
    y_spd = data.twist.linear.y
    current_vx = x_spd
    current_vy = y_spd
    current_vz = data.twist.linear.z
    current_speed = math.sqrt(math.pow(x_spd, 2) + math.pow(y_spd, 2))
def angvel_callback(data):
    global current_p,current_q,current_r

    current_p = data.angular_velocity.x
    current_q = data.angular_velocity.y
    current_r = data.angular_velocity.z

def get_bearing(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1
    bearing_rad = math.atan2(delta_y, delta_x)
    return bearing_rad
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

def create_mpc():
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # Define state variables
    x = model.set_variable(var_type='_x', var_name='x')
    y = model.set_variable(var_type='_x', var_name='y')
    z = model.set_variable(var_type='_x', var_name='z')
    vx = model.set_variable(var_type='_x', var_name='vx')
    vy = model.set_variable(var_type='_x', var_name='vy')
    vz = model.set_variable(var_type='_x', var_name='vz')
    roll = model.set_variable(var_type='_x', var_name='roll')
    pitch = model.set_variable(var_type='_x', var_name='pitch')
    yaw = model.set_variable(var_type='_x', var_name='yaw')
    p = model.set_variable(var_type='_x', var_name='p')
    q = model.set_variable(var_type='_x', var_name='q')
    r = model.set_variable(var_type='_x', var_name='r')
    # Define control inputs
    u1 = model.set_variable(var_type='_u', var_name='u1')  # Thrust
    u2 = model.set_variable(var_type='_u', var_name='u2')  # Roll rate control
    u3 = model.set_variable(var_type='_u', var_name='u3')  # Pitch rate control
    u4 = model.set_variable(var_type='_u', var_name='u4')  # Yaw rate control

    # Physical constants
    g = 9.81  # gravity
    m = 1.5  # mass of the quadrotor
    Ixx = 0.008 # moment of inertia around x-axis
    Iyy = 0.015 # moment of inertia around y-axis
    Izz = 0.017  # moment of inertia around z-axis
    J_tp = 0.0065025  # total rotational inertia of the propellers
    omega = 8 # rad/s**2
    # Define system dynamics based on the given equations
    model.set_rhs('x', vx)
    model.set_rhs('y', vy)
    model.set_rhs('z', vz)

    # Updated dynamics for translational accelerations
    model.set_rhs('vx', (cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw)) * u1 / m)
    model.set_rhs('vy', (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw)) * u1 / m)
    model.set_rhs('vz', -g + cos(roll) * cos(pitch) * u1 / m)

    # Kinematic equations for orientation
    model.set_rhs('roll', p + q * sin(roll) * tan(pitch) + r * cos(roll) * tan(pitch))
    model.set_rhs('pitch', q * cos(roll) - r * sin(roll))
    model.set_rhs('yaw', q * sin(roll) / cos(pitch) + r * cos(roll) / cos(pitch))

    # Updated dynamics for rotational velocities
    model.set_rhs('p', q * r * (Iyy - Izz) / Ixx - J_tp / Ixx * q * omega + u2 / Ixx)
    model.set_rhs('q', p * r * (Izz - Ixx) / Iyy + J_tp / Iyy * p * omega + u3 / Iyy)
    model.set_rhs('r', p * q * (Ixx - Iyy) / Izz + u4 / Izz)


    model.setup()

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 20,
        'n_robust': 1,
        't_step': 0.01,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 2,
        'collocation_ni': 2,
        'store_full_solution': True,
        'nlpsol_opts': {
            'ipopt.print_level': 0,
            'ipopt.tol': 1e-8,
            'ipopt.max_iter': 100,
            'ipopt.mu_strategy': 'adaptive',
        }
    }

    mpc.set_param(**setup_mpc)

    # Define Q and R matrices
    Q = np.diag([40, 40, 40, 1, 1, 1, 40, 40, 40, 1, 1, 1])  # State weights
    R = np.diag([0.7, 100.0,100.0, 100.0])  # Input weights

    # Define cost function

    mterm = (model.x['x'] - x_set) ** 2 * Q[0, 0] + \
            (model.x['y'] - y_set) ** 2 * Q[1, 1] + \
            (model.x['z'] - alt_setpoint) ** 2 * Q[2, 2] + \
            (model.x['vx'] - 0) ** 2 * Q[3, 3] + \
            (model.x['vy'] - 0) ** 2 * Q[4, 4] + \
            (model.x['vz'] - 0) ** 2 * Q[5, 5] + \
            (model.x['roll'] - 0) ** 2 * Q[6, 6] + \
            (model.x['pitch'] - 0) ** 2 * Q[7, 7] + \
            (model.x['yaw'] - bearing) ** 2 * Q[8, 8] + \
            (model.x['p'] - 0) ** 2 * Q[9, 9] + \
            (model.x['q'] - 0) ** 2 * Q[10, 10] + \
            (model.x['r'] - 0) ** 2 * Q[11, 11]

    lterm = (model.x['x'] - x_set) ** 2 * Q[0, 0] + \
            (model.x['y'] - y_set) ** 2 * Q[1, 1] + \
            (model.x['z'] - alt_setpoint) ** 2 * Q[2, 2] + \
            (model.x['vx'] - 0) ** 2 * Q[3, 3] + \
            (model.x['vy'] - 0) ** 2 * Q[4, 4] + \
            (model.x['vz'] - 0) ** 2 * Q[5, 5] + \
            (model.x['roll'] - 0) ** 2 * Q[6, 6] + \
            (model.x['pitch'] - 0) ** 2 * Q[7, 7] + \
            (model.x['yaw'] - bearing) ** 2 * Q[8, 8] + \
            (model.x['p'] - 0) ** 2 * Q[9, 9] + \
            (model.x['q'] - 0) ** 2 * Q[10, 10] + \
            (model.x['r'] - 0) ** 2 * Q[11, 11]

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm( u1[0,0],u2=R[1,1],u3=R[2, 2], u4=R[3, 3])

    # Define constraints
    mpc.bounds['lower', '_u', 'u1'] = 0
    mpc.bounds['upper', '_u', 'u1'] = 1
    mpc.bounds['lower', '_u', 'u2'] = -500
    mpc.bounds['upper', '_u', 'u2'] = 500
    mpc.bounds['lower', '_u', 'u3'] = -500
    mpc.bounds['upper', '_u', 'u3'] = 500
    mpc.bounds['lower', '_u', 'u4'] = -500
    mpc.bounds['upper', '_u', 'u4'] = 500

    mpc.setup()

    # Set initial guess for control inputs
    mpc.x0 = np.zeros((12,))  # Initial state guess
    mpc.u0 = np.zeros((4,))   # Initial control input guess
    mpc.set_initial_guess()

    return mpc

def mpc_control(event):
    global mpc, current_alt, current_x, current_y, current_yaw, euler_distance, bearing, x_set, y_set, alt_setpoint, current_vx, current_vy, current_vz, current_roll, current_pitch, current_p, current_q, current_r

    # Update the current state vector with the latest values
    x0 = np.array([current_x, current_y, current_alt, 
                   current_vx, current_vy, current_vz, 
                   current_roll, current_pitch,current_yaw,
                   current_p, current_q, current_r])

    # Set the initial state of the MPC
    mpc.x0 = x0

    # Make a step with the MPC
    u0 = mpc.make_step(x0)

    # Control inputs
    thrust = u0[0]
    roll_rate = u0[1]
    pitch_rate = u0[2]
    yaw_rate = u0[3]
    

    # Log current status
    rospy.loginfo(f"Altitude: {current_alt} -- {alt_setpoint} || Yaw: {current_yaw} -- {bearing} | Distance: {euler_distance}")
    rospy.loginfo(f"Yaw rate : {yaw_rate} -- {current_r} || Pitch rate : {pitch_rate} -- {current_p} || roll_rate {roll_rate} -- {current_q}")

    rc_msg = rc(pitch_rate, roll_rate, yaw_rate, thrust)  
    pub.publish(rc_msg)


def start():
    global pub, mpc
    arm()
    stab()
    rospy.init_node('yaw_mpc', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)

    rospy.Subscriber('/mavros/global_position/local', Odometry, alt_callback)
    rospy.Subscriber('/mavros/global_position/local', Odometry, yaw_callback)
    rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, speed_callback)

    mpc = create_mpc()

    rospy.Timer(rospy.Duration(0.01), mpc_control)

    rospy.spin()

if __name__ == '__main__':
    start()
