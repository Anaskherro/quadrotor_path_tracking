import rospy
from mavros_msgs.msg import RCOut

def pwm_to_thrust_percentage(pwm):
    pwm_min = 1000
    pwm_max = 2000
    thrust_percentage = (pwm - pwm_min) / (pwm_max - pwm_min) 
    return thrust_percentage
def pwm_to_angular_speed(pwm, pwm_min=1000, pwm_max=2000, omega_min=0, omega_max=500):
    """
    Convert PWM value to angular speed (rad/s).
    """
    omega = ((pwm - pwm_min) / (pwm_max - pwm_min)) * (omega_max - omega_min) + omega_min
    return omega

def rc_out_callback(data):
    pwm_values = data.channels

    # Convert PWM values to thrust percentage
    thrust_percentages = [pwm_to_thrust_percentage(pwm) for pwm in pwm_values[:4]]  # Assuming first 4 channels are motor PWMs
    angular_speeds = [pwm_to_angular_speed(pwm) for pwm in pwm_values[:4]]  # Assuming first 4 channels are motor PWMs
    pwms = [pwm for pwm in pwm_values[:4]]
    
    rospy.loginfo(f"Thrust percentages: {thrust_percentages}")
    rospy.loginfo(f"PWM : {pwms}")
    rospy.loginfo(f"Angular speeds (rad/s): {angular_speeds}")

def listener():
    
    rospy.init_node('rc_out_listener', anonymous=True)
    rospy.Subscriber("/mavros/rc/out", RCOut, rc_out_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
