#!/usr/bin/env python3

from re import I
import sys
import copy
import rospy
import math
import numpy as np
from math import radians
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import cos, pi, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionResult
from gripperControl import *
from panel import *

rospy.init_node("closePanel", anonymous=False)
group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
lid_Store_position = rospy.get_param('lidStorage')
# arucoID = 12
move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])
Inspec_panel = rospy.get_param('tag12')#[0.33565261545991887, -0.2745230369010141, 0.2059755184403631]
GoToLid(lid_Store_position, move_group)
gripperPos("semi_close")
rospy.sleep(4)
PlaceLid(Inspec_panel,move_group)
gripperPos("open")
rospy.sleep(4)
move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])
