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
import datetime
import json

up1 = [0,-(pi/2), 0, -(pi/2), 0, 0]
#imu1 = [radians(91), radians(-128), radians(115), radians(-97), radians(-116), radians(117)]
imuBoard1 = [radians(17), radians(-125), radians(54), radians(-94), radians(-71), radians(81)]
imu = [radians(-127), radians(-91), radians(-85), radians(-94), radians(99), radians(91)]
imuBoard = [radians(45), radians(-99), radians(62), radians(-118), radians(-110), radians(113)]
imu1 = [radians(-131.15), radians(-131), radians(45), radians(-157), radians(116), radians(52)]



storageArea1 = [radians(101), radians(-70), radians(-54), radians(-110), radians(136), radians(34)]
storageArea3=[radians(150),radians(-61),radians(-118),radians(-77),radians(92),radians(58)]

box= [radians(157), radians(-42), radians(-120), radians(-52), radians(86), radians(91)]
lid= [radians(157), radians(-68), radians(-95), radians(-57), radians(89), radians(91)]
storageArea2 = [radians(120), radians(-58), radians(-53), radians(-121), radians(107), radians(35)]

up = [0,-(pi/2), 0, -(pi/2), 0, 0]

buttons1 = [radians(64), radians(-156), radians(109), radians(-132), radians(-157), radians(86)]
buttonsBox = [radians(73), radians(-148), radians(81), radians(-52), radians(-157), radians(143)]
#buttons_top_row = [radians(79), radians(-144), radians(98), radians(-131), radians(-169), radians(96)]
buttons_top_row = [radians(-11), radians(-133), radians(102), radians(17), radians(89), radians(-86)]
buttons_middle_row = [radians(-10), radians(-133), radians(104), radians(37), radians(95), radians(-87)]
buttons_bottom_row = [radians(-10), radians(-133), radians(104), radians(53), radians(94), radians(-87)]



#storageArea = [radians(-50), radians(-131), radians(55), radians(-51), radians(-114), radians(91)]
storageArea = [radians(101), radians(-70), radians(-46), radians(-128), radians(113), radians(34)]
home= [radians(0), radians(-120), radians(100), radians(20), radians(90), radians(-90)]






group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('arucoScanner',anonymous=False)
move_group =moveit_commander.MoveGroupCommander(group_name) 
robot = moveit_commander.RobotCommander()

def scan():
    now = move_group.get_current_joint_values()
    rospy.set_param('start', move_group.get_current_joint_values())
    gripperPos("open") 
    sleep(2)

    move_group.go(up)
    move_group.go(storageArea3)
    move_group.go(storageArea1)
    move_group.go(box)
    move_group.go(lid)
    move_group.go(up)

    move_group.go(imuBoard)
    move_group.go(buttonsBox)
    move_group.go(buttons1)
    move_group.go(imuBoard1)
    move_group.go(up)

    move_group.go(imu1)
    move_group.go(up)
    move_group.go(imu)



    move_group.go(buttons_top_row)
    move_group.go(buttons_middle_row)
    move_group.go(buttons_bottom_row)





    move_group.go(now)

    #move_group.go(storageArea)
   # move_group.go(up1)
   # move_group.go(imu1)
    #move_group.go(up1)
   # move_group.go(imuBoard1)
   # move_group.go(up1)
   # move_group.go(buttonsBox1)
   # move_group.go(up1)
   # move_group.go(storageArea1)
    

def nodeKiller(toKill, aruco, override = False):
    isStart = datetime.datetime.now()
    while True:
        diff = ((datetime.datetime.now() - isStart).total_seconds())/60.0
        if len(aruco.aruco)>=14 or diff>=3:            
            sleep(2)
            # move_group.go(home)
            # sleep(4)
            gripperPos("close")
            sleep(2)
            os.system("rosnode kill aruco_detect")

            # try:
            #     with open("aruco_ids.json","w") as outfile:
            #         json.dump(aruco.aruco, outfile)
            #         print("Aruco data saved")
            # except IOError as e:
            #     print("Error writing to the file:", e)

            try:
                rospy.set_param('/tag1', [aruco.aruco[1][0],aruco.aruco[1][1]])
            except:
                rospy.set_param('/tag1', [[0,0,0],[0,0,0,0]])
                aruco.aruco[1] = [[0,0,0],[0,0,0,0]]
            try:     
                rospy.set_param('/tag2', [aruco.aruco[2][0],aruco.aruco[2][1]])
            except:
                rospy.set_param('/tag2', [[0,0,0],[0,0,0,0]])
                aruco.aruco[2] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag3', [aruco.aruco[3][0],aruco.aruco[3][1]])
            except:
                rospy.set_param('/tag3', [[0,0,0],[0,0,0,0]])
                aruco.aruco[3] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag4', [aruco.aruco[4][0],aruco.aruco[4][1]])
            except:
                rospy.set_param('/tag4', [[0,0,0],[0,0,0,0]])
                aruco.aruco[4] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag5', [aruco.aruco[5][0],aruco.aruco[5][1]])
            except:
                rospy.set_param('/tag5', [[0,0,0],[0,0,0,0]])
                aruco.aruco[5] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag6', [aruco.aruco[6][0],aruco.aruco[6][1]])
            except:
                rospy.set_param('/tag6', [[0,0,0],[0,0,0,0]])
                aruco.aruco[6] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag7', [aruco.aruco[7][0],aruco.aruco[7][1]])
            except:
                rospy.set_param('/tag7', [[0,0,0],[0,0,0,0]])
                aruco.aruco[7] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag8', [aruco.aruco[8][0],aruco.aruco[8][1]])
            except:
                rospy.set_param('/tag8', [[0,0,0],[0,0,0,0]])
                aruco.aruco[8] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag9', [aruco.aruco[9][0],aruco.aruco[9][1]])
            except:
                rospy.set_param('/tag9', [[0,0,0],[0,0,0,0]])
                aruco.aruco[9] = [[0,0,0],[0,0,0,0]]
            try:
                rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
            except:
                aruco.aruco[10] =   ([0.19091511546601642, 0.2610590673579687, -0.08998876185262991], [-0.005436175436329082, -0.004139026343300572, 0.7079299498368842, 0.706249603597412])
                rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
                
            try:
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            except:
                aruco.aruco[11] =   ([0.4142767622791573, 0.34664772988934184, 0.5153305618864111], [-0.5622667274017764, 0.408915326157582, 0.4222457089405716, -0.5816811365100378])
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            try:
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            except:
                aruco.aruco[12] =   ([0.33652467558540594, -0.2749759883220442, 0.20541202056776836], [-0.41392029095141303, 0.5599210594303042, 0.5785001056771537, -0.4248482407597626])
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            try:
                rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
            except:
                aruco.aruco[13] =   ([0.37273712509483437, -0.23370562648394416, 0.24112094772921994], [0.009042158836901643, -0.008851421818332948, -0.8029254314194612, 0.595945168010644])
                rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
            try:
                rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])
            except:
                aruco.aruco[14] =  ([0.2627005704280358, -0.18725236367743253, -0.14572040839702463], [-0.004944659506009925, -0.0049388815186685335, -0.7114809434512608, 0.7026706375660068])
                rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])
            rospy.wait_for_service("erc_aruco_score")
            try:
                service_proxy = rospy.ServiceProxy('erc_aruco_score',ErcAruco)
                # os.system("rosnode kill aruco_detect")
                # create object of the request type for the Service (14 tags)
                service_msg = ErcArucoRequest()

                service_msg.tag1=aruco.aruco[1][0]
             #   rospy.set_param('/tag1', [aruco.aruco[1][0],aruco.aruco[1][1]])
                service_msg.tag2=aruco.aruco[2][0]
            #rospy.set_param('/tag2', [aruco.aruco[2][0],aruco.aruco[2][1]])
                service_msg.tag3=aruco.aruco[3][0]
               # rospy.set_param('/tag3', [aruco.aruco[3][0],aruco.aruco[3][1]])
                service_msg.tag4=aruco.aruco[4][0]
              #  rospy.set_param('/tag4', [aruco.aruco[4][0],aruco.aruco[4][1]])
                service_msg.tag5= aruco.aruco[5][0]
                #rospy.set_param('/tag5', [aruco.aruco[5][0],aruco.aruco[5][1]])
                service_msg.tag6= aruco.aruco[6][0]
               # rospy.set_param('/tag6', [aruco.aruco[6][0],aruco.aruco[6][1]])
                service_msg.tag7=aruco.aruco[7][0]
                #rospy.set_param('/tag7', [aruco.aruco[7][0],aruco.aruco[7][1]])
                service_msg.tag8=aruco.aruco[8][0]
                #rospy.set_param('/tag8', [aruco.aruco[8][0],aruco.aruco[8][1]])
                service_msg.tag9=aruco.aruco[9][0]
                #rospy.set_param('/tag9', [aruco.aruco[9][0],aruco.aruco[9][1]])
                service_msg.tag10=aruco.aruco[10][0]
                #rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
                service_msg.tag11=aruco.aruco[11][0]
               # rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
                service_msg.tag12=aruco.aruco[12][0]
               # rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
                service_msg.tag13=aruco.aruco[13][0]
                #rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
                service_msg.tag14=aruco.aruco[14][0]
                #rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])

                # call the service with your message through service proxy
                # and receive the response, which happens to be your score
                service_response = service_proxy(service_msg)
                print(f"You received score {service_response.score}")
                #move_group.go([0,-(pi/2), 0, -(pi/2), 0, 0])
                #gripperPos("close")
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
            sleep(1)
            # move_group.go(up)
            os.system("rosnode kill arucoScanner")

aruco = arucoPos()
from threading import Timer
t1 = threading.Thread(target=scan)
t2 = threading.Thread(target=aruco.exec)
t3 = threading.Thread(target= nodeKiller, args=(t2,aruco))
t1.start()
t2.start()
t3.start()
t2.join()
t1.join()
t3.join()

