#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from erc_aruco_msg.srv import ErcArucoRequest, ErcArucoResponse, ErcAruco
from math import pi, radians
from time import sleep
import threading
import os
from arucoPos import *
from pressButton import *

up = [0,-(pi/2), 0, -(pi/2), 0, 0]
imu = [radians(91), radians(-128), radians(115), radians(-97), radians(-116), radians(117)]
imuBoard = [radians(78), radians(-98), radians(61), radians(23), radians(147), radians(-105)]
buttonsBox = [radians(93), radians(13), radians(-89), radians(-104), radians(144), radians(90)]
storageArea = [radians(98), radians(-45), radians(-106), radians(-101), radians(110), radians(44)]

group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('arucoScanner',anonymous=False)
move_group =moveit_commander.MoveGroupCommander(group_name) 
robot = moveit_commander.RobotCommander()

def scan():
    rospy.set_param('start', move_group.get_current_joint_values())
    move_group.go(up)
    move_group.go(imu)
    move_group.go(up)
    move_group.go(imuBoard)
    move_group.go(up)
    move_group.go(buttonsBox)
    move_group.go(up)
    move_group.go(storageArea)

def nodeKiller(toKill, aruco):
    while True:
        if len(aruco.aruco)>=13:
            sleep(1)
            os.system("rosnode kill aruco_detect")
            rospy.set_param('/tag1', [aruco.aruco[1][0],aruco.aruco[1][1]])
            rospy.set_param('/tag2', [aruco.aruco[2][0],aruco.aruco[2][1]])
            rospy.set_param('/tag3', [aruco.aruco[3][0],aruco.aruco[3][1]])
            rospy.set_param('/tag4', [aruco.aruco[4][0],aruco.aruco[4][1]])
            rospy.set_param('/tag5', [aruco.aruco[5][0],aruco.aruco[5][1]])
            rospy.set_param('/tag6', [aruco.aruco[6][0],aruco.aruco[6][1]])
            rospy.set_param('/tag7', [aruco.aruco[7][0],aruco.aruco[7][1]])
            rospy.set_param('/tag8', [aruco.aruco[8][0],aruco.aruco[8][1]])
            rospy.set_param('/tag9', [aruco.aruco[9][0],aruco.aruco[9][1]])
            rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
            rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
            rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])

            print(rospy.get_param_names())
            rospy.wait_for_service("erc_aruco_score")
            try:
                service_proxy = rospy.ServiceProxy('erc_aruco_score',ErcAruco)
                os.system("rosnode kill aruco_detect")
                # create object of the request type for the Service (14 tags)
                service_msg = ErcArucoRequest()

                service_msg.tag1=aruco.aruco[1][0]
                rospy.set_param('/tag1', [aruco.aruco[1][0],aruco.aruco[1][1]])
                service_msg.tag2=aruco.aruco[2][0]
                rospy.set_param('/tag2', [aruco.aruco[2][0],aruco.aruco[2][1]])
                service_msg.tag3=aruco.aruco[3][0]
                rospy.set_param('/tag3', [aruco.aruco[3][0],aruco.aruco[3][1]])
                service_msg.tag4=aruco.aruco[4][0]
                rospy.set_param('/tag4', [aruco.aruco[4][0],aruco.aruco[4][1]])
                service_msg.tag5= aruco.aruco[5][0]
                rospy.set_param('/tag5', [aruco.aruco[5][0],aruco.aruco[5][1]])
                service_msg.tag6== aruco.aruco[6][0]
                rospy.set_param('/tag6', [aruco.aruco[6][0],aruco.aruco[6][1]])
                service_msg.tag7== aruco.aruco[7][0]
                rospy.set_param('/tag7', [aruco.aruco[7][0],aruco.aruco[7][1]])
                service_msg.tag8== aruco.aruco[8][0]
                rospy.set_param('/tag8', [aruco.aruco[8][0],aruco.aruco[8][1]])
                service_msg.tag9== aruco.aruco[9][0]
                rospy.set_param('/tag9', [aruco.aruco[9][0],aruco.aruco[9][1]])
                service_msg.tag10=aruco.aruco[10][0]
                rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
                service_msg.tag11=aruco.aruco[11][0]
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
                service_msg.tag12=aruco.aruco[12][0]
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
                service_msg.tag13=aruco.aruco[13][0]
                rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
                service_msg.tag14=aruco.aruco[14][0]
                rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])

                # call the service with your message through service proxy
                # and receive the response, which happens to be your score
                service_response = service_proxy(service_msg)
                print(f"You received score {service_response.score}")
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
            sleep(1)
            move_group.go(up)
            print(rospy.get_param_names)
            os.system("rosnode kill arucoScanner")
def runIt():
    aruco = arucoPos()
    t1 = threading.Thread(target=scan)
    t2 = threading.Thread(target=aruco.exec)
    t3 = threading.Thread(target= nodeKiller, args=(t2,aruco))
    t2.start()
    t1.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()