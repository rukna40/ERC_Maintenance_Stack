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
from panel import *
from gripperControl import *

rospy.init_node("openPanel", anonymous=False)
group_name = "manipulator"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# arucoID = 12
Inspec_panel=rospy.get_param('tag12')#[[0.33652467558540594, -0.2749759883220442, 0.20541202056776836], [-0.41392029095141303, 0.5599210594303042, 0.5785001056771537, -0.4248482407597626]]
# arucoID = 14

Inspec_lid_storage= rospy.get_param('tag14')#[[0.2627005704280358, -0.18725236367743253, -0.14572040839702463],[-0.004944659506009925, -0.0049388815186685335, -0.7114809434512608, 0.7026706375660068]]

gripperPos("open")
goToUp(move_group)
lid_original_position = PickLid(Inspec_panel, move_group)
gripperPos("semi_close")
lid_store_position=StoreLid(Inspec_lid_storage, move_group)
rospy.sleep(4)
pose = ([lid_store_position.position.x, lid_store_position.position.y, lid_store_position.position.z],[lid_store_position.orientation.x, lid_store_position.orientation.y, lid_store_position.orientation.z, lid_store_position.orientation.w])
rospy.set_param('lidStorage',pose)
rospy.sleep(4)
