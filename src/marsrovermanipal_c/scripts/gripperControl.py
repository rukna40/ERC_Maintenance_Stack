#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String
import time

def gripperPos(str):
    pub = rospy.Publisher('/gripper_command', String, queue_size=10)
    pub.publish(String(str))
    time.sleep(.5)
    pub.publish(String(str))
    time.sleep(.5)
    pub.publish(String(str))
    time.sleep(.5)
    pub.publish(String(str))
    time.sleep(.5)
