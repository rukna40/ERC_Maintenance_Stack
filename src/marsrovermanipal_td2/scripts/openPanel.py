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
Inspec_panel=[[0.4543321460818059, -0.27308862481984303, 0.20675289603645802], [-0.41393664848981404, 0.5613857914087025, 0.5813523239313731, -0.41895702252094025]]#rospy.get_param('tag12')
# arucoID = 14

Inspec_lid_storage=[[0.378707047206316, -0.18283937512228204, -0.1344733802356643], [-0.04107175299356068, -0.050058705746646635, -0.7181035289303752, 0.692794482781875]]#rospy.get_param('tag14')

gripperPos("open")
rospy.sleep(2)
goToUp(move_group)
rospy.sleep(3)
lid_original_position = PickLid(Inspec_panel, move_group)
rospy.sleep(2)
gripperPos("semi_close")
rospy.sleep(3)
lid_store_position=StoreLid(Inspec_panel,Inspec_lid_storage, move_group)
pose = ([lid_store_position.position.x, lid_store_position.position.y, lid_store_position.position.z],[lid_store_position.orientation.x, lid_store_position.orientation.y, lid_store_position.orientation.z, lid_store_position.orientation.w])
rospy.set_param('lidStorage',pose)
rospy.sleep(4)
