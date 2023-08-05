#!/usr/bin/env python3

from re import I, X
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
from moveit_msgs.msg import MoveGroupActionResult, RobotState
import time
from gripperControl import *


#rospy.init_node("InspectionPanel", anonymous=True)
group_name = "manipulator"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
gripper_group = moveit_commander.MoveGroupCommander("gripper")
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


# # arucoID = 14
# Inspec_lid_storage = [0.2636325217102241-0.04, -
#                       0.1863268703997993, -0.14519731884030188]
# Inspec_lid_storage_qua = [0.2627005704280358, -0.18725236367743253, -0.14572040839702463, -
#                           0.004944659506009925, -0.0049388815186685335, -0.7114809434512608, 0.7026706375660068]
# # arucoID = 13
# Inspec_lid = [0.37265761073396486, -0.23365844633784216, 0.2410547809570061]
# Inspec_lid_qua = [0.37273712509483437, -0.23370562648394416, 0.24112094772921994,
#                   0.009042158836901643, -0.008851421818332948, -0.8029254314194612, 0.595945168010644]
# # arucoID = 12
# Inspec_panel = [0.33565261545991887, -0.2745230369010141, 0.2059755184403631]
# Inspec_panel_qua = [0.33652467558540594, -0.2749759883220442, 0.20541202056776836, -
#                     0.41392029095141303, 0.5599210594303042, 0.5785001056771537, -0.4248482407597626]
# # rpy needed
# rpy_r = [3.119479487810088, 0.3924501498248416, 2.7395040785837512]
# rpy_d = [178.73300892959315433, 22.485737254296026322, 156.96202166211372742]


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


def goToUp(move_group):

    up = [radians(0), radians(-90), radians(0),
          radians(-90), radians(0), radians(90)]
    move_group.go(up)




def PickLid(box,move_group):
    aboveStoragePre = [radians(152), radians(-83), radians(-61),
                    radians(-104), radians(96), radians(80)]
    
    aboveStorage = [radians(150), radians(-80), radians(-75),
                    radians(-95), radians(95), radians(80)]
    move_group.go(aboveStoragePre)
    rospy.sleep(2)
    move_group.go(aboveStorage)
    
    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)

    eular = quaternion_to_euler(box[1][0], box[1][1], box[1][2], box[1][3])
    eular = list(eular)
    eular[0] = eular[0] + radians(90)
    eular[1] = 0.4
    eular[2] = eular[2] - radians(90)

    qua = euler_to_quaternion(eular[0], eular[1], eular[2])
    print(qua)

    wpose.position.x = box[0][0] + 0.01
    wpose.position.y = box[0][1] - 0.0032
    wpose.position.z = box[0][2] + 0.125
    wpose.orientation.x = qua[0]
    wpose.orientation.y = qua[1]
    wpose.orientation.z = qua[2]
    wpose.orientation.w = qua[3]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("Reached Lid Potition")
    wpose = move_group.get_current_pose(end).pose
    print(wpose)
    print("")
    
    # lid_position = move_group.get_current_joint_values()
    # print("")
    # print(lid_position)
    # print("")

    return [wpose.position.x, wpose.position.y, wpose.position.z]


def StoreLid(lidStorage,move_group):

    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)

    eular = quaternion_to_euler(lidStorage[1][0], lidStorage[1][1], lidStorage[1][2], lidStorage[1][3])
    yaw = eular[2]

    

    waypoints = []
    wpose.position.z += 0.02
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("up")
    rospy.sleep(3)
    waypoints = []

    wpose.position.x = lidStorage[0][0] #- 0.0679 #+ 0.1925*sin(wrist_1_joint_angle)#*cos(yaw-wrist_3_joint_angle)
    wpose.position.y = lidStorage[0][1] #- 0.0679 #- 0.1925*sin(wrist_1_joint_angle)#*sin(yaw-wrist_3_joint_angle)

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("back")

    # waypoints = []
    # wpose.position.z = lidStorage[0][2] + 0.2
    
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = move_group.compute_cartesian_path(
    #     waypoints, 0.01, 0.0, True)
    # move_group.execute(plan, wait=True)
    # print("down")
    # print("")

    waypoints = []
    joint_values = move_group.get_current_joint_values()
    rospy.loginfo("Joint values: {}".format(joint_values))

    wrist_1_joint_index = move_group.get_joints().index("wrist_1_joint")
    wrist_1_joint_angle = joint_values[wrist_1_joint_index]
    wrist_3_joint_index = move_group.get_joints().index("wrist_3_joint")
    wrist_3_joint_angle = joint_values[wrist_3_joint_index]
    
    wpose.position.x = lidStorage[0][0] - 0.0679 -0.02 #- 0.1925*sin(wrist_1_joint_angle)*cos(yaw-wrist_3_joint_angle)
    wpose.position.y = lidStorage[0][1] - 0.0679 #+ 0.1925*sin(wrist_1_joint_angle)*sin(yaw-wrist_3_joint_angle)

    print(0.1925*sin(wrist_1_joint_angle)*cos(yaw - wrist_3_joint_angle))
    print(0.1925*sin(wrist_1_joint_angle)*sin(yaw - wrist_3_joint_angle))
    
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    
    waypoints = []
    wpose.position.z = lidStorage[0][2] + 0.1
    
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    end = move_group.get_end_effector_link()
    lid_position = move_group.get_current_pose(end).pose
    print("Reached Lid Storage Position")
    print(lid_position)

    return lid_position


def GoToScan(box,move_group):

    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)

    waypoints = []
    wpose.position.z = box[0][2] + 0.125

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    waypoints = []
    wpose.position.x = box[0][0] - 0.05
    wpose.position.y = box[0][1] - 0.0032

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("Scanning Position Reached")
    print("")


def GoToLid(lid_position,lid_store_position,move_group):
    move_group.go([radians(134), radians(-106), radians(-143),radians(-2), radians(102), radians(72.1)])
    rospy.sleep(3)

    lid_pos_eular = quaternion_to_euler(lid_position[1][0], lid_position[1][1], lid_position[1][2], lid_position[1][3])
    lid_store_eular = quaternion_to_euler(lid_store_position[1][0], lid_store_position[1][1], lid_store_position[1][2], lid_store_position[1][3])
    lid_yaw = lid_pos_eular[2]
    yaw=lid_yaw-radians(90)

    qua = euler_to_quaternion(lid_store_eular[0], lid_store_eular[1], yaw)

    joint_values = move_group.get_current_joint_values()
    rospy.loginfo("Joint values: {}".format(joint_values))

    wrist_1_joint_index = move_group.get_joints().index("wrist_1_joint")
    wrist_1_joint_angle = joint_values[wrist_1_joint_index]
    wrist_3_joint_index = move_group.get_joints().index("wrist_3_joint")
    wrist_3_joint_angle = joint_values[wrist_3_joint_index]
    
    waypoints = []
    end = move_group.get_end_effector_link()
    wpose = move_group.get_current_pose(end).pose
    # print(wpose)
    rospy.loginfo("Yaw: {}".format(wrist_1_joint_angle))
    wpose.position.x = lid_position[0][0] + 0.0249*cos(lid_yaw) - 0.1925*sin(wrist_1_joint_angle)*cos(yaw-wrist_3_joint_angle)
    wpose.position.y = lid_position[0][1] + 0.0529*sin(lid_yaw) + 0.1925*sin(wrist_1_joint_angle)*sin(yaw-wrist_3_joint_angle)
    wpose.position.z = lid_position[0][2] + 0.125 - 0.035 #- 0.1 + 0.1925*cos(wrist_1_joint_angle)
    wpose.orientation.x = qua[0]
    wpose.orientation.y = qua[1]
    wpose.orientation.z = qua[2]
    wpose.orientation.w = qua[3]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    
    # move_group.go(lid_position)
    print("Reached Lid Storage Position")
    print("")


def PlaceLid(lid_position,move_group):

    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z = lid_position[0][2] + 0.15

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)
    print("up")

    eular = quaternion_to_euler(lid_position[1][0], lid_position[1][1], lid_position[1][2], lid_position[1][3])
    eular = list(eular)
    eular[0] = eular[0] + radians(90)
    eular[1] = 0.4
    eular[2] = eular[2] - radians(90)

    qua = euler_to_quaternion(eular[0], eular[1], eular[2])

    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.x = lid_position[0][0] + 0.01
    wpose.position.y = lid_position[0][1] - 0.0032
    wpose.orientation.x = qua[0]
    wpose.orientation.y = qua[1]
    wpose.orientation.z = qua[2]
    wpose.orientation.w = qua[3]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    waypoints = []
    wpose = move_group.get_current_pose().pose

    wpose.position.z = lid_position[0][2] + 0.12

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, True)
    move_group.execute(plan, wait=True)

    print("Lid Placed")


# def main():
#     try:
#         goToUp()
#         lid_original_position = PickLid(Inspec_panel_qua)
#         # rospy.sleep(2)
#         gripperPos("semi_close")
#         rospy.sleep(4)
#         lid_Store_position = StoreLid(Inspec_lid_storage)
#         gripperPos("open")
#         rospy.sleep(4)
#         GoToScan(Inspec_panel)

#         # # here will be scan and press the button code

#         GoToLid(lid_Store_position)
#         gripperPos("semi_close")
#         rospy.sleep(4)
#         PlaceLid(Inspec_panel, lid_original_position)
#         gripperPos("open")
#         rospy.sleep(4)

#         # check()
#     except rospy.ROSInterruptException:
#         return
#     except KeyboardInterrupt:
#         return


# if __name__ == "__main__":

#     main()
