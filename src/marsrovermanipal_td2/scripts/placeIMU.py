#!/usr/bin/env python3

import sys
import rospy
import math
import moveit_commander
from gripperControl import *
from math import degrees, radians, pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
import tf
from copy import deepcopy

rospy.init_node("placeIMU", anonymous=True)
group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander("gripper")
robot = moveit_commander.RobotCommander()

IMU_orientation = float(rospy.get_param('angle'))
Panel = rospy.get_param("tag11") #[[0.40932626600050137,0.34655615615960345,0.5167615000923605],[0.5384723206812556,-0.39617854501926364,-0.4329243277084065,0.5968683977392999]]


def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def quaternion_to_euler(x, y, z, w):
    euler = tf.transformations.euler_from_quaternion([x, y, z, w])
    return euler

def PlaceImu(Panel, move_group):
    gripperPos("semi_open")
    rospy.sleep(2)

    aboveStorage = [radians(7), radians(-97), radians(81), radians(15), radians(64), radians(-90)]
    move_group.go(aboveStorage)

    waypoints = []
    end_effector_link = move_group.get_end_effector_link()
    current_pose = move_group.get_current_pose(end_effector_link).pose

    euler_angles = quaternion_to_euler(Panel[1][0], Panel[1][1], Panel[1][2], Panel[1][3])
    euler_angles = list(euler_angles)
    print(degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2]))
    euler_angles[0] = pi/2
    euler_angles[1] = pi/2
    euler_angles[2] += pi
    print(degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2]))
    quaternion = euler_to_quaternion(euler_angles[0], euler_angles[1], euler_angles[2])
    print(quaternion)

    current_pose.position.x = Panel[0][0] - 0.08520
    current_pose.position.y = Panel[0][1] - 0.09954
    current_pose.position.z = Panel[0][2] - 0.15
    current_pose.orientation.x = quaternion[0]
    current_pose.orientation.y = quaternion[1]
    current_pose.orientation.z = quaternion[2]
    current_pose.orientation.w = quaternion[3]
    waypoints.append(deepcopy(current_pose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0, True)

    # Define velocity scaling factor
    velocity_scaling = 0.5  # Adjust this value to control the speed (0.5 = 50% speed)
    move_group.set_max_velocity_scaling_factor(velocity_scaling)  # Set velocity scaling factor

    move_group.execute(plan, wait=True)
    print("")
    print("Reached near IMU panel")

    waypoints = []
    current_pose = move_group.get_current_pose(end_effector_link).pose

    euler_angles = quaternion_to_euler(Panel[1][0], Panel[1][1], Panel[1][2], Panel[1][3])
    euler_angles = list(euler_angles)
    print(degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2]))
    euler_angles[0] = pi/2
    euler_angles[1] = radians(IMU_orientation)
    euler_angles[2] += pi
    print(degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2]))
    quaternion = euler_to_quaternion(euler_angles[0], euler_angles[1], euler_angles[2])
    print(quaternion)

    current_pose.orientation.x = quaternion[0]
    current_pose.orientation.y = quaternion[1]
    current_pose.orientation.z = quaternion[2]
    current_pose.orientation.w = quaternion[3]
    waypoints.append(deepcopy(current_pose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0, True)

    move_group.execute(plan, wait=True)
    print("")
    print("Taken IMU orientation")

    waypoints = []
    current_pose = move_group.get_current_pose(end_effector_link).pose
    current_pose.position.x += Panel[0][0] + 0.056 - 0.01332 - 0.43 - 0.045
    current_pose.position.y += Panel[0][1] - 0.042 - 0.00433 - 0.3
    waypoints.append(deepcopy(current_pose))
    print(current_pose)

    # Add extra waypoints to move the end effector further
    extra_waypoints = deepcopy(waypoints)
    extra_waypoints[-1].position.x += 0.05
    extra_waypoints[-1].position.y += 0.0
    extra_waypoints[-1].position.z += 0.0
    waypoints.extend(extra_waypoints)

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
    move_group.execute(plan)

    current_pose = move_group.get_current_pose().pose
    print(current_pose)
    gripperPos("open")
    rospy.sleep(2)
    home= [radians(0), radians(-120), radians(100), radians(20), radians(90), radians(-90)]
    move_group.go(home)
    rospy.sleep(2)

try:
    PlaceImu(Panel, move_group)
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted by user")
except Exception as e:
    rospy.logerr("An error occurred: %s" % str(e))
