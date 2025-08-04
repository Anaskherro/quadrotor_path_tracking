import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import do_mpc
from casadi import *

# Initialize global variables for current state
current_alt = 0.0
current_x = 0.0
current_y = 0.0
current_pitch = 0.0
current_roll = 0.0
current_vx = 0.0
current_vy = 0.0
current_vz = 0.0
current_p = 0.0  # Roll rate
current_q = 0.0  # Pitch rate

# Setpoints
alt_setpoint = 10.0
x_set = 10.0
y_set = 10.0

# RC Message initialization
rc_msg = OverrideRCIn()
rc_min = 1000
rc_max = 2000

def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        response = arming_cl(value=True)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Arming failed: %s" % e)

def stab():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = change_mode(custom_mode="ACRO")
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        print("Set mode failed: %s" % e)

def alt_callback(data):
    global current_alt, current_x, current_y
    current_alt = data.pose.position.z
    current_x = data.pose.position.x
    current_y = data.pose.position.y

def imu_callback(data):
    global current_pitch, current_roll, current_p, current_q
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, _) = euler_from_quaternion(orientation_list)
    current_roll = roll
    current_pitch = pitch
    current_p = data.angular_velocity.x
    current_q = data.angular_velocity.y

def create_position_mpc():
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # Define state variables for position
    x = model.set_variable(var_type='_x', var_name='x')
    y = model.set_variable(var_type='_x', var_name='y')
    vx = model.set_variable(var_type='_x', var_name='vx')
    vy = model.set_variable(var_type='_x', var_name='vy')

    # Define control inputs for position
    roll_rate = model.set_variable(var_type='_u', var_name='roll_rate')
    pitch_rate = model.set_variable(var_type='_u', var_name='pitch_rate')

    # Define system dynamics for position
    model.set_rhs('x', vx)
    model.set_rhs('y', vy)
    model.set_rhs('vx', roll_rate)  # Simplified dynamics
    model.set_rhs('vy', pitch_rate)  # Simplified dynamics

    model.setup()

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 10,
        't_step': 0.1,
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

    # Define objective
    Q = np.diag([1, 1, 0.1, 0.1])
    R = np.diag([0.01, 0.01])

    mterm = (model.x['x'] - x_set)**2 * Q[0, 0] + (model.x['y'] - y_set)**2 * Q[1, 1]
    lterm = (model.x['x'] - x_set)**2 * Q[0, 0] + (model.x['y'] - y_set)**2 * Q[1, 1]

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(roll_rate=R[0, 0], pitch_rate=R[1, 1])

    # Define constraints
    mpc.bounds['lower', '_u', 'roll_rate'] = -math.radians(360)
    mpc.bounds['upper', '_u', 'roll_rate'] = math.radians(360)
    mpc.bounds['lower', '_u', 'pitch_rate'] = -math.radians(360)
    mpc.bounds['upper', '_u', 'pitch_rate'] = math.radians(360)

    mpc.setup()
    
    # Set initial guess for position MPC
    mpc.x0 = np.array([0.0, 0.0, 0.0, 0.0])  # Initial state guess
    mpc.u0 = np.array([0.0, 0.0])           # Initial control input guess
    mpc.set_initial_guess()

    return mpc

def create_altitude_mpc():
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # Define state variables for altitude
    z = model.set_variable(var_type='_x', var_name='z')
    vz = model.set_variable(var_type='_x', var_name='vz')

    # Define control inputs for altitude
    thrust = model.set_variable(var_type='_u', var_name='thrust')

    # Define system dynamics for altitude
    model.set_rhs('z', vz)
    model.set_rhs('vz', thrust)  # gravity compensation

    model.setup()

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 10,
        't_step': 0.1,
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

    # Define objective
    Q = np.diag([1, 0.1])
    R = np.diag([0.01])

    mterm = (model.x['z'] - alt_setpoint)**2 * Q[0, 0]
    lterm = (model.x['z'] - alt_setpoint)**2 * Q[0, 0]

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(thrust=R[0, 0])

    # Define constraints
    mpc.bounds['lower', '_u', 'thrust'] = 0
    mpc.bounds['upper', '_u', 'thrust'] = 1

    mpc.setup()
    
    # Set initial guess for altitude MPC
    mpc.x0 = np.array([0.0, 0.0])  # Initial state guess
    mpc.u0 = np.array([0.0])       # Initial control input guess (mid-throttle)
    mpc.set_initial_guess()

    return mpc

def mapp(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

def position_control(event):
    global position_mpc, current_x, current_y
    x0 = np.array([current_x, current_y, current_vx, current_vy])  # Current state
    position_mpc.x0 = x0
    u0 = position_mpc.make_step(x0)
    roll_rate = u0[0]
    pitch_rate = u0[1]

    rc_msg.channels[0] = int(mapp(math.degrees(roll_rate), -360, 360, rc_min, rc_max))
    rc_msg.channels[1] = int(mapp(math.degrees(pitch_rate), -360, 360, rc_min, rc_max))
    pub.publish(rc_msg)

def altitude_control(event):
    global altitude_mpc, current_alt
    z0 = np.array([current_alt, current_vz])  # Current state
    altitude_mpc.x0 = z0
    uz = altitude_mpc.make_step(z0)

    thrust = uz[0]
    rc_msg.channels[2] = int(mapp(thrust, 0, 1, rc_min, rc_max))
    pub.publish(rc_msg)

def start():
    global pub, position_mpc, altitude_mpc

    rospy.init_node('drone_mpc', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, alt_callback)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)

    position_mpc = create_position_mpc()
    altitude_mpc = create_altitude_mpc()

    rospy.Timer(rospy.Duration(0.01), position_control)
    rospy.Timer(rospy.Duration(0.01), altitude_control)

    rospy.spin()

if __name__ == '__main__':
    arm()
    stab()
    start()
