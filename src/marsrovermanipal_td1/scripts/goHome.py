#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from math import pi

group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('goHome',anonymous=False)
move_group =moveit_commander.MoveGroupCommander(group_name) 
robot = moveit_commander.RobotCommander()
#move_group.go(rospy.get_param('start'))
group_variables = move_group.get_current_joint_values()
group_variables[0] = 0
group_variables[1] = -2.094395
group_variables[2] = 1.745329
group_variables[3] = 0.349065
group_variables[4] = 1.570796
group_variables[5] = -1.570796
move_group.set_joint_value_target(group_variables)
plan = move_group.go(wait=True)
