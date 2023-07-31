#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from pressButton import *
from gripperControl import *
from math import pi

group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('buttonPresser',anonymous=False)
move_group =moveit_commander.MoveGroupCommander(group_name) 
robot = moveit_commander.RobotCommander()
press_buttons = list(map(int,rospy.get_param('~tags').split(',')))
for i in press_buttons:
    press(i, move_group)
gripperPos("open")
move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])
