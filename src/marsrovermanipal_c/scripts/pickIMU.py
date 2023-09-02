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
from math import cos, pi, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionResult
import time
import geometry_msgs.msg
from copy import deepcopy


rospy.init_node("pickIMU", anonymous=True)
group_name = "manipulator"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander("gripper")
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


IMU_Module = rospy.get_param('tag10') # -0.707, 0.707, 0, 0 xyzw
Panel = rospy.get_param("tag11")

# arucoID = 10
##IMU_Module = [0.19091, 0.26106, -0.08999,-0.00543, -0.0, 0.70793, 0.70793]
# arucoID = 11
#Panel = [0.41428, 0.346645, 0.51533,-0.56227, 0.40892, 0.42225, -0.58168]

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



def PickImu(IMU_Module,move_group):
    
    gripperPos("open")
    rospy.sleep(1)
    
    # move the arm to the home position
    group_variables = move_group.get_current_joint_values()
    group_variables[0] = 0
    group_variables[1] = -2.094395
    group_variables[2] = 1.745329
    group_variables[3] = 0.349065
    group_variables[4] = 1.570796
    group_variables[5] = -1.570796
    move_group.set_joint_value_target(group_variables)
    plan = move_group.go(wait=True)

    aboveIMU = [radians(35), radians(-81), radians(114),
                    radians(-123), radians(-91), radians(35)]
    move_group.go(aboveIMU)

    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose

    eular = quaternion_to_euler(IMU_Module[1][0], IMU_Module[1][1], IMU_Module[1][2], IMU_Module[1][3])
    eular = list(eular)
    eular[0] = 0
    eular[1] = pi
    #eular[2] = 

    qua = euler_to_quaternion(eular[0], eular[1], eular[2])
    print(qua)

    wpose.position.x = IMU_Module[0][0]
    wpose.position.y = IMU_Module[0][1] 
    wpose.position.z = 0 
    
    wpose.orientation.x = qua[0]
    wpose.orientation.y = qua[1]
    wpose.orientation.z = qua[2]
    wpose.orientation.w = qua[3]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    
    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    
    wpose.position.z = IMU_Module[0][2] + 0.04 #0.044 
    
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    
   
    print("")
    print("Reached IMU Position")
    

    gripperPos("semi_open")
    rospy.sleep(1)
    
    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    
    wpose.position.z = 0.1 
    
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("")
    print("Picked up IMU")
    rospy.sleep(3)

PickImu(IMU_Module,move_group)