#!/usr/bin/env python3

from re import I, X
import sys
import copy
import rospy
import math
import numpy as np
from gripperControl import *
from math import degrees, radians, cos, pi, sin
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Bool
from math import cos, pi, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionResult
import time
import geometry_msgs.msg
from copy import deepcopy


rospy.init_node("placeIMU", anonymous=True)
group_name = "manipulator"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander("gripper")
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


# # arucoID = 10
# IMU_Module = [0.19091, 0.26106, -0.08999,-0.00543, -0.0, 0.70793, 0.70793]
# # arucoID = 11
#Panel = [0.41428, 0.346645, 0.51533,-0.56227, 0.40892, 0.42225, -0.58168]

IMU_orientation = float(rospy.get_param('angle'))
Panel = rospy.get_param("tag11")


def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return radians(X), radians(Y), radians(Z)


def PlaceImu(Panel, move_group):
    
    gripperPos("semi_open")
    rospy.sleep(2)
    
    aboveStorage = [radians(7), radians(-97), radians(81),
                    radians(15), radians(64), radians(-90)]
    move_group.go(aboveStorage)

    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose

    eular = quaternion_to_euler(Panel[1][0], Panel[1][1], Panel[1][2], Panel[1][3])
    eular = list(eular)
    print(degrees(eular[0]), degrees(eular[1]), degrees(eular[2]))
    eular[0] = pi/2
    eular[1] = pi/2
    eular[2] = eular[2] + pi
    print(degrees(eular[0]), degrees(eular[1]), degrees(eular[2]))
    qua = euler_to_quaternion(eular[0], eular[1], eular[2])
    print(qua)

    wpose.position.x = Panel[0][0]-0.08520
    wpose.position.y = Panel[0][1]-0.09954
    wpose.position.z = Panel[0][2]-0.15
    wpose.orientation.x = qua[0]
    wpose.orientation.y = qua[1]
    wpose.orientation.z = qua[2]
    wpose.orientation.w = qua[3]
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("")
    print("Reached near IMU panel")

    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose

    eular = quaternion_to_euler(Panel[1][0], Panel[1][1], Panel[1][2], Panel[1][3])
    eular = list(eular)
    print(degrees(eular[0]), degrees(eular[1]), degrees(eular[2]))
    eular[0] = pi/2
    eular[1] = radians(IMU_orientation)
    eular[2] = eular[2] + pi
    print(degrees(eular[0]), degrees(eular[1]), degrees(eular[2]))
    qua = euler_to_quaternion(eular[0], eular[1], eular[2])
    print(qua)

    wpose.orientation.x = qua[0]
    wpose.orientation.y = qua[1]
    wpose.orientation.z = qua[2]
    wpose.orientation.w = qua[3]
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("")
    print("Taken IMU orientation")

    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    wpose.position.x += Panel[0][0] + 0.056 - 0.01332 -0.43
    wpose.position.y += Panel[0][1] - 0.042 - 0.00433 -0.3
    waypoints.append(copy.deepcopy(wpose))
    print(wpose)
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan)
    
    wpose = move_group.get_current_pose().pose
    print(wpose)
    gripperPos("open")
    rospy.sleep(2)

    print("")
    print("********IMU Placed********")
    print("")

    # try:
    #     service_proxy = rospy.ServiceProxy('erc_score', Bool)
    #     service_response = service_proxy(True)
    # except rospy.ServiceException as e:
    #     print(f"Service call failed: {e}")
    move_group.go([0, -(pi/2), 0, -(pi/2), 0, 0])


PlaceImu(Panel, move_group)
