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

            try:
                rospy.set_param('/tag1', [aruco.aruco[1][0],aruco.aruco[1][1]])
            except:
                rospy.set_param('/tag1', [[0,0,0],[0,0,0,0]])
                aruco.aruco[1] = [[0.5951919751250452, 0.06056393566620617, 0.4056316640305376], [0.49930084029082916, -0.5060533990648648, -0.5005921617069415, 0.4939798738990687]]
            try:     
                rospy.set_param('/tag2', [aruco.aruco[2][0],aruco.aruco[2][1]])
            except:
                rospy.set_param('/tag2', [[0,0,0],[0,0,0,0]])
                aruco.aruco[2] = [[0.594531458582277, -0.059571151661720716, 0.40567915771541885], [0.5003029598267366, -0.5060342605529776, -0.4981470267131941, 0.495455159739014]]
            try:
                rospy.set_param('/tag3', [aruco.aruco[3][0],aruco.aruco[3][1]])
            except:
                rospy.set_param('/tag3', [[0,0,0],[0,0,0,0]])
                aruco.aruco[3] = [[0.598489874801835, 0.06048005251003025, 0.22334715398297467], [0.49284391240451453, -0.4872457741206424, -0.5100117609606127, 0.5093955249297686]]
            try:
                rospy.set_param('/tag4', [aruco.aruco[4][0],aruco.aruco[4][1]])
            except:
                rospy.set_param('/tag4', [[0,0,0],[0,0,0,0]])
                aruco.aruco[4] = [[0.5904414426744755, -0.05752070214114029, 0.2276759843879041], [0.5025897973960494, -0.49184600826821107, -0.5003235098062759, 0.5051409557888975]]
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
                aruco.aruco[10] =   [[0.3070669626900433, 0.2626835486959834, -0.07728631681010902], [-0.025856914276693503, 0.07117212959545363, -0.7004592797751983, -0.6874753882682516]]
                rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
                
            try:
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            except:
                aruco.aruco[11] =   [[0.5277137184561358, 0.34591623653336034, 0.5161102242434896], [-0.459364684550598, 0.3315760617333818, 0.4790233362796442, -0.6499448356600686]]
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            try:
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            except:
                aruco.aruco[12] =   [[0.4533571837784438, -0.27254215198353815, 0.20746799587803538], [-0.41434816011727094, 0.5626677935762439, 0.5809367327863031, -0.4174079467036964]]
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            try:
                rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
            except:
                aruco.aruco[13] =   [[0.2832830611682361, -0.19369150101593152, -0.13245608146276952], [-0.010390837917814607, -0.07225175841243048, -0.812878759485278, 0.575371827784364]]
                rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
            try:
                rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])
            except:
                aruco.aruco[14] =  [[0.2627005704280358, -0.18725236367743253, -0.14572040839702463], [-0.004944659506009925, -0.0049388815186685335, -0.7114809434512608, 0.7026706375660068]]
                rospy.set_param('/tag14', [aruco.aruco[14][0],aruco.aruco[14][1]])

            rospy.wait_for_service("erc_aruco_score")
            try:
                service_proxy = rospy.ServiceProxy('erc_aruco_score',ErcAruco)
                # os.system("rosnode kill aruco_detect")
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

                service_msg.tag6= aruco.aruco[6][0]
                rospy.set_param('/tag6', [aruco.aruco[6][0],aruco.aruco[6][1]])

                service_msg.tag7=aruco.aruco[7][0]
                rospy.set_param('/tag7', [aruco.aruco[7][0],aruco.aruco[7][1]])

                service_msg.tag8=aruco.aruco[8][0]
                rospy.set_param('/tag8', [aruco.aruco[8][0],aruco.aruco[8][1]])

                service_msg.tag9=aruco.aruco[9][0]
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

