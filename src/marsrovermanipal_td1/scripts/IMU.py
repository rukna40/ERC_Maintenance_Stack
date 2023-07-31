#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
#from gripperControl import *

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


def main():
    # initialising moveit commanders
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("imu_pick_and_place_objective", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # intialising movement groups
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    #gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # arm_group.set_pose_reference_frame('base_link')
    end_effector_link = arm_group.get_end_effector_link()
    print(end_effector_link)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # move the arm to the home position
    group_variables = arm_group.get_current_joint_values()
    group_variables[0] = 0
    group_variables[1] = -2.094395
    group_variables[2] = 1.745329
    group_variables[3] = 0.349065
    group_variables[4] = 1.570796
    group_variables[5] = -1.570796
    arm_group.set_joint_value_target(group_variables)
    plan = arm_group.go(wait=True)

    # get the home pose data and intialise it as starting pose
    start_pose = arm_group.get_current_pose(end_effector_link).pose
    print(start_pose)
    quat = [start_pose.orientation.x, start_pose.orientation.y,
            start_pose.orientation.z, start_pose.orientation.w]
    eu = euler_from_quaternion(quat)
    eu = [degrees(eu[0]), degrees(eu[1]), degrees(eu[2])]
    print(eu)

    # prepare to grab the IMU
    grab_pose_goal = [IMU_Module[0], IMU_Module[1], IMU_Module[2]+0.044, 0, pi, pi/2]
    arm_group.set_pose_target(grab_pose_goal)
    plan = arm_group.go(wait=True)

    # grab the IMU by closing the gripper
    #gripperPos("semi_open")
    rospy.sleep(2)

    # bring IMU near the panel 5cm above the velcro
    panel_pose_goal = [Panel[0]-0.08520, Panel[1]-0.09954, Panel[2]-0.15, pi/2, 0, 3/5*pi]
    arm_group.set_pose_target(panel_pose_goal)
    plan = arm_group.go(wait=True)

    #orient the IMU
    orientation_pose_goal = [Panel[0]-0.08520, Panel[1]-0.09954, Panel[2]-0.15, pi/2, pi/180*IMU_orientation, 3/5*pi]
    arm_group.set_pose_target(orientation_pose_goal)
    plan = arm_group.go(wait=True)

    #place the IMU by moving towards the velcro panel
    waypoints = []
    wpose = arm_group.get_current_pose().pose
    wpose.position.x += Panel[0]-0.06599
    wpose.position.y += Panel[1]-0.09330
    waypoints.append(copy.deepcopy(wpose))
    (plan,fraction)=arm_group.compute_cartesian_path(waypoints,0.001,0.0,True)
    arm_group.execute(plan)
    
    #release the IMU
    #gripperPos("open")
    rospy.sleep(2)

    #return to home position
    group_variables = arm_group.get_current_joint_values()
    group_variables[0] = 0
    group_variables[1] = -2.094395
    group_variables[2] = 1.745329
    group_variables[3] = 0.349065
    group_variables[4] = 1.570796
    group_variables[5] = -1.570796
    arm_group.set_joint_value_target(group_variables)
    plan = arm_group.go(wait=True)

    rospy.sleep(1)


if __name__ == "__main__":
    main()
