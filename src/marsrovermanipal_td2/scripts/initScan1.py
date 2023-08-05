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
            move_group.go(home)
            sleep(4)
            gripperPos("close")
            sleep(2)
            os.system("rosnode kill aruco_detect")

            try:
                rospy.set_param('/tag1', [aruco.aruco[1][0],aruco.aruco[1][1]])
            except:
                rospy.set_param('/tag1', [[0.5945420015966167, 0.10075593849046427, 0.47599191086925635], [0.49630462062846936, -0.5005686583886008, -0.5031869352390497, 0.49986928961651766]])
                aruco.aruco[1] = [[0.5945420015966167, 0.10075593849046427, 0.47599191086925635], [0.49630462062846936, -0.5005686583886008, -0.5031869352390497, 0.49986928961651766]]
            try:     
                rospy.set_param('/tag2', [aruco.aruco[2][0],aruco.aruco[2][1]])
            except:
                rospy.set_param('/tag2', [[0.5948931228057182, 0.000582446550333023, 0.47610093456111113], [0.4998670666690623, -0.5023406125777059, -0.4994180211896935, 0.49829311336247456]])
                aruco.aruco[2] = [[0.5948931228057182, 0.000582446550333023, 0.47610093456111113], [0.4998670666690623, -0.5023406125777059, -0.4994180211896935, 0.49829311336247456]]
            try:
                rospy.set_param('/tag3', [aruco.aruco[3][0],aruco.aruco[3][1]])
            except:
                rospy.set_param('/tag3', [[0.5951815060189813, -0.09970403465111782, 0.4761986711911297], [0.4974061098001771, -0.504672163931142, -0.502265111093438, 0.4955925847851726]])
                aruco.aruco[3] = [[0.5951815060189813, -0.09970403465111782, 0.4761986711911297], [0.4974061098001771, -0.504672163931142, -0.502265111093438, 0.4955925847851726]]
            try:
                rospy.set_param('/tag4', [aruco.aruco[4][0],aruco.aruco[4][1]])
            except:
                rospy.set_param('/tag4', [[0.5942372912033229, 0.10070551092414977, 0.3156858563512934], [0.4912364010742958, -0.4989198615793202, -0.5078205889554392, 0.5018753931866364]])
                aruco.aruco[4] = [[0.5942372912033229, 0.10070551092414977, 0.3156858563512934], [0.4912364010742958, -0.4989198615793202, -0.5078205889554392, 0.5018753931866364]]
            try:
                rospy.set_param('/tag5', [aruco.aruco[5][0],aruco.aruco[5][1]])
            except:
                rospy.set_param('/tag5', [[0.5926234076560966, 0.0011468426747610473, 0.31613980554838716], [0.5035969690904079, -0.49376441530358905, -0.49748480363170733, 0.5050699605554433]])
                aruco.aruco[5] = [[0.5926234076560966, 0.0011468426747610473, 0.31613980554838716], [0.5035969690904079, -0.49376441530358905, -0.49748480363170733, 0.5050699605554433]]
            try:
                rospy.set_param('/tag6', [aruco.aruco[6][0],aruco.aruco[6][1]])
            except:
                rospy.set_param('/tag6', [[0.5915568562005649, -0.09776427269881235, 0.3164212337522694], [0.5009111553180512, -0.4980841768410011, -0.49966724290624887, 0.501331041952346]])
                aruco.aruco[6] = [[0.5915568562005649, -0.09776427269881235, 0.3164212337522694], [0.5009111553180512, -0.4980841768410011, -0.49966724290624887, 0.501331041952346]]
            try:
                rospy.set_param('/tag7', [aruco.aruco[7][0],aruco.aruco[7][1]])
            except:
                rospy.set_param('/tag7', [[0.612333045664636, 0.10150869488020958, 0.14289609917343926], [0.15698186754135066, -0.17981891264311525, 0.08182912160940109, -0.10119962068302102]])
                aruco.aruco[7] = [[0.612333045664636, 0.10150869488020958, 0.14289609917343926], [0.15698186754135066, -0.17981891264311525, 0.08182912160940109, -0.10119962068302102]]
            try:
                rospy.set_param('/tag8', [aruco.aruco[8][0],aruco.aruco[8][1]])
            except:
                rospy.set_param('/tag8', [[0.604004250330678, -0.0014385744632648984, 0.148636068663133], [0.4993209137407204, -0.47139181442768396, -0.5117907520910463, 0.5156165698886137]])
                aruco.aruco[8] = [[0.604004250330678, -0.0014385744632648984, 0.148636068663133], [0.4993209137407204, -0.47139181442768396, -0.5117907520910463, 0.5156165698886137]]
            try:
                rospy.set_param('/tag9', [aruco.aruco[9][0],aruco.aruco[9][1]])
            except:
                rospy.set_param('/tag9', [[0.5872732381280872, -0.09493440435128772, 0.16046097977516222], [0.5075574246718685, -0.49370860822968465, -0.49628022212418343, 0.5023377124992996]])
                aruco.aruco[9] = [[0.5872732381280872, -0.09493440435128772, 0.16046097977516222], [0.5075574246718685, -0.49370860822968465, -0.49628022212418343, 0.5023377124992996]]
            try:
                rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
            except:
                aruco.aruco[10] =   [[0.3067593068150112, 0.26295886547619335, -0.07622942616533704], [-0.025884108887387645, 0.06809387368772815, -0.7056008681141006, -0.6937170204105669]]
                rospy.set_param('/tag10', [aruco.aruco[10][0],aruco.aruco[10][1]])
                
            try:
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            except:
                aruco.aruco[11] =   [[0.516652588705103, 0.3432811903353892, 0.5199324602371316], [-0.38588141432803924, 0.30505822272539324, 0.5229019306757239, -0.676953124115954]]
                rospy.set_param('/tag11', [aruco.aruco[11][0],aruco.aruco[11][1]])
            try:
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            except:
                aruco.aruco[12] =   [[0.45532066398072185, -0.2736415102861426, 0.20607065243266723], [-0.4178537280719124, 0.5607408493919221, 0.5792230845957833, -0.41885796436614553]]
                rospy.set_param('/tag12', [aruco.aruco[12][0],aruco.aruco[12][1]])
            try:
                rospy.set_param('/tag13', [aruco.aruco[13][0],aruco.aruco[13][1]])
            except:
                aruco.aruco[13] =   [[0.27900224641099136, -0.19364302602803057, -0.11144469913897928], [-0.05870413697719699, -0.0009646231551708965, -0.8166871251848803, 0.5702666925006381]]
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
