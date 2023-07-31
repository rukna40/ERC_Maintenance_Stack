#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
import math
from gripperControl import *
from geometry_msgs.msg import Pose
from copy import deepcopy
from math import pi, degrees
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveGroupActionResult

IMU_Module = [0.19, 0.26, -0.089]  # -0.707, 0.707, 0, 0 xyzw
Panel = [0.409, 0.342, 0.515]
IMU_orientation = 45

def euler_to_quaternion(roll, pitch, yaw):

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
#e =(0, pi, (pi/2))
#eular =list(e)
#qua = euler_from_quaternion(eular[0],eular[1],eular[2])
#print(qua)
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("imu_pick_and_place_objective", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander("manipulator")


    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    
    goal = move_group.get_current_pose().pose
#     goal = Pose();
    goal.position.x = IMU_Module[0]
    goal.position.y = IMU_Module[1]
    goal.position.z = IMU_Module[2]+0.044
    goal.orientation.x=-0.707
    goal.orientation.y=0.707
    goal.orientation.z=0
    goal.orientation.w=0
    #goal.orientation = posec.orientation
    # prepare to grab the IMU
    move_group.set_pose_target(goal)
    plan = move_group.go(wait=True)

    # grab the IMU by closing the gripper
    #gripperPos("semi_open")
    #rospy.sleep(2)

    # # bring IMU near the panel 5cm above the velcro
    # goal.position.x =Panel[0]-0.08520
    # goal.position.y =Panel[1]-0.09954
    # goal.position.z= Panel[2]-0.15
    # goal.orientation = move_group.get_current_pose().orientation
    # move_group.set_pose_target(goal)
    # plan = move_group.go(wait=True)

    # #orient the IMU
    # goal.orientation = move_group.get_current_pose().orientatio
    # orientation_pose_goal = [Panel[0]-0.08520, Panel[1]-0.09954, Panel[2]-0.15, pi/2, pi/180*IMU_orientation, 3/5*pi]
    # move_group.set_pose_target(orientation_pose_goal)
    # plan = move_group.go(wait=True)

    # #place the IMU by moving towards the velcro panel
    # waypoints = []
    # wpose = move_group.get_current_pose().pose
    # wpose.position.x += Panel[0]-0.06599
    # wpose.position.y += Panel[1]-0.09330
    # waypoints.append(copy.deepcopy(wpose))
    # (plan,fraction)=move_group.compute_cartesian_path(waypoints,0.001,0.0,True)
    # move_group.execute(plan)
    
    # #release the IMU
    # #gripperPos("open")
    # #rospy.sleep(2)

    # #return to home position
    # group_variables = move_group.get_current_joint_values()
    # group_variables[0] = 0
    # group_variables[1] = -2.094395
    # group_variables[2] = 1.745329
    # group_variables[3] = 0.349065
    # group_variables[4] = 1.570796
    # group_variables[5] = -1.570796
    # move_group.set_joint_value_target(group_variables)
    # plan = move_group.go(wait=True)

    # rospy.sleep(1)


if __name__ == "__main__":
    main()
