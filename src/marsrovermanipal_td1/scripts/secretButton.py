#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from pressButton import *
from gripperControl import *
from math import pi

group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('secretButton',anonymous=False)
move_group =moveit_commander.MoveGroupCommander(group_name) 
robot = moveit_commander.RobotCommander()
gripperPos("close")
rospy.sleep(3)
try:
    i = rospy.get_param('hiddenButton')
except:
    i=1
press(i, move_group)
rospy.sleep(3)
gripperPos("open")
rospy.sleep(2)
move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])