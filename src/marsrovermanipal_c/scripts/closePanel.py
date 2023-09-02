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
# move_group.set_max_velocity_scaling_factor(0.1)
# move_group.set_max_acceleration_scaling_factor(0.1)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
lid_pos = rospy.get_param('tag13')#[[0.14753648982289141,-0.19949166597950677,-0.13262711210629644],[-0.006622104699622651,-0.009061137655766759,-0.7044306145019219,0.7096841218923916]]
# arucoID = 12
move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])
rospy.sleep(3)
gripperPos("open")
rospy.sleep(2)
Inspec_panel =rospy.get_param('tag12')#[[0.4543321460818059, -0.27308862481984303, 0.20675289603645802], [-0.41393664848981404, 0.5613857914087025, 0.5813523239313731, -0.41895702252094025]]
lid_store_pos=rospy.get_param('lidStorage')
GoToLid(lid_pos,lid_store_pos, move_group)
rospy.sleep(2)
gripperPos("semi_close")
rospy.sleep(3)
PlaceLid(Inspec_panel,move_group)
rospy.sleep(2)
gripperPos("open")
rospy.sleep(3)
move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])