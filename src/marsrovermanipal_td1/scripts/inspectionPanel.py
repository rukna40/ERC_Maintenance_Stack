#!/usr/bin/env python3

import copy
import csv
import math
from shutil import move
import sys
from math import cos, pi, radians, sin
from re import I, M
from gripperControl import *
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveGroupActionResult
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node("InspectionPanel", anonymous=True)
group_name = "manipulator"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

move_group.set_planning_time(15)
move_group.allow_replanning(True)
move_group.clear_path_constraints()

# Set the right arm reference frame
# move_group.set_pose_reference_frame("base_link")


# arucoID = 14
Inspec_lid_storage = [0.2636325217102241-0.04, -0.1863268703997993, -0.14519731884030188]
Inspec_lid_storage_qua = [0.2627005704280358, -0.18725236367743253, -0.14572040839702463, -0.004944659506009925, -0.0049388815186685335, -0.7114809434512608, 0.7026706375660068]
# arucoID = 13
Inspec_lid = [0.37265761073396486, -0.23365844633784216, 0.2410547809570061]
Inspec_lid_qua = [0.37273712509483437, -0.23370562648394416, 0.24112094772921994,0.009042158836901643, -0.008851421818332948, -0.8029254314194612, 0.595945168010644]
# arucoID = 12
Inspec_panel = [0.33565261545991887, -0.2745230369010141, 0.2059755184403631]
Inspec_panel_qua = [0.33652467558540594, -0.2749759883220442, 0.20541202056776836, -0.41392029095141303, 0.5599210594303042, 0.5785001056771537, -0.4248482407597626]
# rpy needed
rpy_r = [3.119479487810088, 0.3924501498248416, 2.7395040785837512]
rpy_d = [178.73300892959315433, 22.485737254296026322, 156.96202166211372742]


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


def goToUp():

    # home = [radians(0), radians(-120), radians(100), radians(20), radians(90), radians(-90)]
    # move_group.go(home)

    up = [radians(0), radians(-90), radians(0),radians(-90), radians(0), radians(80)]
    move_group.go(up)

    # aboveStorage = [radians(155), radians(-87), radians(-81),
    #                 radians(-68), radians(92), radians(88)]
    # move_group.go(aboveStorage)

    # mid = [radians(91), radians(-84), radians(-108), radians(-66), radians(122), radians(21)]
    # move_group.go(mid)


def GripperState(gripperState):
    pub = rospy.Publisher('/gripper_command', String, queue_size=1)
    rospy.sleep(1)
    for _ in range(500):
        pub.publish(gripperState)
    print(gripperState)


def PickLid(box):
    
    move_group.clear_path_constraints()

    eular = quaternion_to_euler(box[3], box[4], box[5], box[6])
    print(eular)
    
    #got to middle potition to get the required joint angle
    mid = [radians(153), radians(-16), radians(-130), radians(-90), radians(96), radians(92)]
    move_group.go(mid)



    end = move_group.get_end_effector_link()
    pose = move_group.get_current_pose(end)
    
    constraints = Constraints()
    constraints.name = "Keep gripper as it is"

    # Create an orientation constraint for the right gripper
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    orientation_constraint.link_name = move_group.get_end_effector_link()
    #orientation_constraint.orientation.w = 1.0
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 3.14
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    constraints.orientation_constraints.append(orientation_constraint)
    move_group.set_path_constraints(constraints)

        
    constraints = Constraints()
    constraints.name = "Rotate from back"

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    orientation_constraint.link_name = "shoulder_link"
    orientation_constraint.orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
    orientation_constraint.absolute_x_axis_tolerance = 3.14
    orientation_constraint.absolute_y_axis_tolerance = 3.14
    orientation_constraint.absolute_z_axis_tolerance = pi/2
    orientation_constraint.weight = 1.0

    constraints.orientation_constraints.append(orientation_constraint)
    move_group.set_path_constraints(constraints)
    
    
    # match the orientationn of gripper to lid 
    midPoint = [0.07, -0.14, 0.22, pi, 0.6, eular[2] - radians(90)]
    move_group.set_pose_target(midPoint)
    move_group.go(wait=True)
    
    
    move_group.clear_path_constraints()
    
    
    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)

    wpose.position.x = box[0] + 0.01
    wpose.position.y = box[1] - 0.0032
    wpose.position.z = box[2] + 0.125
    

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    eular = list(eular)

    # z = eular[2]-radians(90)
    # rpy =(pi, 0.6, z)
    # move_group.set_rpy_target(rpy)

    print("Reached Lid Potition")
    wpose = move_group.get_current_pose(end).pose
    print(wpose)
    print("")

    return (wpose.position.x, wpose.position, wpose.position.z)


def StoreLid(lidStorage):

    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)

    waypoints = []
    wpose.position.z += 0.02
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("up")

    waypoints = []
    wpose.position.x = lidStorage[0] - 0.068
    wpose.position.y = lidStorage[1] - 0.068

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("back")

    waypoints = []
    wpose.position.z = lidStorage[2] + 0.1

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("down")
    print("")

    lid_position = move_group.get_current_joint_values()
    print("Reached Lid Stoage Position")
    print(lid_position)
    print("")

    return lid_position


def GoToScan(box):

    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)

    waypoints = []
    wpose.position.z = box[2] + 0.125

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    waypoints = []
    wpose.position.x = box[0] + 0.005
    wpose.position.y = box[1] - 0.0032

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("Sanning Position Reached")
    print("")


def GoToLid(lid_position):

    move_group.go(lid_position)
    print("Reached Lid Storage Position")
    print("")


def PlaceLid(box):

    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z = box[2] + 0.2

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("up")

    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.x = box[0] + 0.01
    wpose.position.y = box[1] - 0.0032

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    waypoints = []
    wpose = move_group.get_current_pose().pose

    wpose.position.z = box[2] + 0.125

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    print("Lid Placed")


def main():
    try:
        goToUp()
        lid_original_position = PickLid(Inspec_panel_qua)
        gripperPos("semi_close")
        lid_Store_position = StoreLid(Inspec_lid_storage)
        gripperPos("open")
        GoToScan(Inspec_panel)

        # # here will be scan and press the button code

        GoToLid(lid_Store_position)
        gripperPos("semi_close")
        PlaceLid(Inspec_panel_qua)
        gripperPos("open")

        # check()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":

    main()
