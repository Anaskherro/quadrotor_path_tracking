#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool


rc_msg = OverrideRCIn()
rc_min=1000
rc_max=2000
roll_max=45
pitch_max=45


def arm():
        print("\nArming")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
                arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                response = arming_cl(value = True)
                rospy.loginfo(response)
        except rospy.ServiceException as e:
                print("Arming failed: %s" %e)

def stab():
        print ("\nSetting Mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
                change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = change_mode(custom_mode="STABILIZE")

                rospy.loginfo(response)
        except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
def auto():
        print ("\nSetting Mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:

                change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = change_mode(custom_mode="AUTO")
                rospy.loginfo(response)
        except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)

def rc(pitch, roll, yaw, throttle) :
        rc_msg.channels[0]=int(mapp(roll,-roll_max,roll_max,rc_min,rc_max)) 
        rc_msg.channels[1]=int(mapp(pitch,-pitch_max,pitch_max,rc_min,rc_max)) 
        rc_msg.channels[2]=int(mapp(throttle,0,1,rc_min,rc_max)) 
        rc_msg.channels[3]=int(mapp(yaw,-1,1,rc_min,rc_max)) 
        return rc_msg

def mapp(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

def callback(data):

    roll = -roll_max * data.axes[0]
    pitch = pitch_max * data.axes[1]
    throttle = (1 - data.axes[2])/2
    yaw = - data.axes[3]
    print("#####################")
    print("roll :"+ str(roll))
    print("pitch :"+ str(pitch))
    print("throttle :"+ str(throttle))
    print("yaw :"+ str(yaw))
    print("#####################")

    rc_msg=rc(pitch, roll, yaw, throttle)

    pub.publish(rc_msg)



def start():
    arm()
    stab()
    global pub
    rospy.init_node("joystick")
    pub = rospy.Publisher('/mavros/rc/override',OverrideRCIn, queue_size=1)
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()

if __name__ == '__main__':

    start()