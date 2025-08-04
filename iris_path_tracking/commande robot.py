#! /usr/bin/env python

import rospy
import mavros
from std_msgs.msg import String,Header
from geometry_msgs.msg import Twist,TwistStamped   
import time
from mavros_msgs.msg import OverrideRCIn,RCIn
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time
channel=13

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(channel, GPIO.OUT)



pub = None
th=50
st=70
stp_th=10
st_min=30

st_max=75
state_description=""

th_min=500
th_max=2188
th_mean=1500

str_min=600
str_max=2200
str_mean=1400

rc_msg = OverrideRCIn()
s=5


seuil=10
rcc=2006
def mapp(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

def rc(a,b) :
        rc_msg.channels[0]=int(mapp(a,-90,90,str_min,str_max)) 
        rc_msg.channels[0]=int(mapp(a,-90,90,str_min,str_max)) 
        #rc_msg.channels[7] = c
        return rc_msg

def clbk(msg):
        global rcc
        rcc=msg.channels[7]
        print("rcc ",rcc)

def clbk_laser(msg):
    global regions
    global mp
    regions = {
        'right':  min(min(msg.ranges[:71]), seuil),
        'fright': min(min(msg.ranges[72:127]), seuil),
        'front':  min(min(msg.ranges[128:236]), seuil),
        'fleft':  min(min(msg.ranges[237:291]), seuil),
        'left':   min(min(msg.ranges[292:362]), seuil),
    }
    mp=min(msg.ranges)
    take_action(regions,mp)

def arm():
        print "\nArming"
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
                arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                response = arming_cl(value = True)
                rospy.loginfo(response)
        except rospy.ServiceException as e:
                print("Arming failed: %s" %e)


def manu():
        print "\nSetting Mode"
        rospy.wait_for_service('/mavros/set_mode')
        try:
                change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = change_mode(custom_mode="MANUAL")

                rospy.loginfo(response)
        except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
def auto():
        print "\nSetting Mode"
        rospy.wait_for_service('/mavros/set_mode')
        try:

                change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                response = change_mode(custom_mode="AUTO")
                rospy.loginfo(response)
        except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)

arm()
manuu=False
autoo=False
def take_action(regions,mp):
        global rc_msg
        global state_description 
        global manuu
        global autoo
        GPIO.output(channel, GPIO.LOW)

        #print(mp)
        if(rcc==2006):
                if(mp<s):
                        GPIO.output(channel, GPIO.HIGH)
                        print("MODE ANTICOLLISION")
                        if(manuu==False):
                                manu()
                        state_description = ''

                        #if regions['front'] > s and regions['fleft'] > s and regions['fright'] > s:
                        #       state_description = 'case 1 - nothing'
                        #       rc_msg=rc(0,th,2006)
                                
                        if regions['front'] < s and regions['fleft'] > s and regions['fright'] > s:
                                state_description = 'case 2 - front'
                                rc_msg=rc(-st_min,th)

                        elif regions['front'] > s and regions['fleft'] > s and regions['fright'] < s:
                                state_description = 'case 3 - fright'
                                rc_msg=rc(-st_min,th)
                        elif regions['front'] > s and regions['fleft'] < s and regions['fright'] > s:
                                state_description = 'case 4 - fleft'
                                rc_msg=rc(st_min,th)

                        elif regions['front'] < s and regions['fleft'] > s and regions['fright'] < s:
                                state_description = 'case 5 - front and fright'
                                rc_msg=rc(-st_max,th)

                        elif regions['front'] < s and regions['fleft'] < s and regions['fright'] > s:
                                state_description = 'case 6 - front and fleft'
                                rc_msg=rc(st_max,th)

                        else:
                                rc_msg=rc(0,10)
                                state_description = 'stop case'
                                rospy.loginfo(regions)
                        rate = rospy.Rate(10)
                        pub.publish(rc_msg)
                        rospy.loginfo(rc_msg)
                        rospy.loginfo(state_description)
                        rate.sleep()

                else:
                        manuu=False
                        if(autoo==False):
                                print("AUTO")
                                print(autoo)
                                auto()
                                autoo=True
                        GPIO.output(channel, GPIO.LOW)
        else:
                print("Not AUTO + ANTI")
                GPIO.output(channel, GPIO.LOW)

                manuu=False
                autoo=False

        '''
        elif regions['front'] < s and regions['fleft'] < s and regions['fright'] < s:
                state_description = 'case 7 - front and fleft and fright'
                rc_msg=get("GAUCHE",st)

        '''

'''
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
'''
def main():
    global pub
    
    rospy.init_node('reading_laser')
    pub = rospy.Publisher('/mavros/rc/override',OverrideRCIn, queue_size=10)
    sub2=rospy.Subscriber('/mavros/rc/in' ,RCIn, clbk)    
    sub = rospy.Subscriber('/rscan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()

