#!/usr/bin/env python3
import sys
import rospy
import tf
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import geometry_msgs.msg
import moveit_commander 
import random
import copy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Transform
from math import pi
import numpy as np
#from buttons import *
import moveit_msgs.msg
class arucoPos:
    aruco = {}
    pose = {}
    br = None
    ls = None
    aruco_temp = {1:[[],[]],2:[[],[]],3:[[],[]],4:[[],[]],5:[[],[]],6:[[],[]],7:[[],[]],8:[[],[]],9:[[],[]],10:[[],[]],11:[[],[]],12:[[],[]],13:[[],[]],14:[[],[]]}    
    aruco_count = {1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0,11:0,12:0,13:0,14:0}
    aruco_sum = {1:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],2:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],3:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],4:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],5:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],6:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],7:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],8:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],9:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],10:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],11:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],12:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],13:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],14:[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]]}

    # def sumTransform(self, transform, newTransform):
    #     if transform==([],[]):
    #         transform=newTransform
    #     else:
    #         transform[0][0] += newTransform[0][0]
    #         transform[0][1] += newTransform[0][1]
    #         transform[0][2] += newTransform[0][2]
    #         transform[1][0] += newTransform[1][0]
    #         transform[1][1] += newTransform[1][1]
    #         transform[1][2] += newTransform[1][2]
    #         transform[1][3] += newTransform[1][3]
    #     return transform
    def sumTransform(self, transform, newTransform):
        if transform==[]:
            transform=newTransform
        else:
            transform[0] += newTransform[0]
            transform[1] += newTransform[1]
            transform[2] += newTransform[2]
            # transform[1][0] += newTransform[1][0]
            # transform[1][1] += newTransform[1][1]
            # transform[1][2] += newTransform[1][2]
            # transform[1][3] += newTransform[1][3]
        return transform
           
    def callback(self,data):
        #print('in callback')
        now = rospy.Time.now()
        for i in range(len(data.transforms)):
            frame = data.transforms[i]
            #print(frame)
            self.pose[frame.fiducial_id] = frame.transform
            translation = (self.pose[frame.fiducial_id].translation.x, self.pose[frame.fiducial_id].translation.y, self.pose[frame.fiducial_id].translation.z)
            rotation = (self.pose[frame.fiducial_id].rotation.x, self.pose[frame.fiducial_id].rotation.y, self.pose[frame.fiducial_id].rotation.z, self.pose[frame.fiducial_id].rotation.w)
            transformer = tf.Transformer()
            self.br.sendTransform(translation, rotation, rospy.Time(0), "aruco" + str(frame.fiducial_id), "camera_link")
            self.ls.waitForTransform("fiducial_" + str(frame.fiducial_id),"base_link",rospy.Time(0), rospy.Duration(15.0))
            new_pose = list(self.ls.lookupTransform("base_link","fiducial_" + str(frame.fiducial_id), rospy.Time(0)))
            self.br.sendTransform(new_pose[0], new_pose[1], rospy.Time(0), "Aruco" + str(frame.fiducial_id), "base_link")
            self.aruco_sum[frame.fiducial_id][0]=self.sumTransform(self.aruco_temp[frame.fiducial_id][0],new_pose[0])     
            self.aruco[frame.fiducial_id]=new_pose  
            self.br.sendTransform(new_pose[0], new_pose[1], rospy.Time(0), "Aruco" + str(frame.fiducial_id), "base_link")
            self.aruco_count[frame.fiducial_id]+=1
            self.aruco_temp[frame.fiducial_id][0] = self.aruco_sum[frame.fiducial_id][0]
            rospy.set_param('/tag'+str(frame.fiducial_id),new_pose)
            print(self.aruco_sum, self.aruco_count)
            # self.pose[frame.fiducial_id]=self.aruco[frame.fiducial_id]
            # if not self.aruco_pos.get(frame.fiducial_id):
            #     self.aruco_pos[frame.fiducial_id] = list(pose)
            # else:
            #     self.aruco_pos[frame.fiducial_id].append(pose)
 
 
    def exec(self):
        print('in exec')
        robot = moveit_commander.RobotCommander()
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.callback)
        while not rospy.is_shutdown() and len(self.aruco)<14:
            rospy.spin()            
            print(self.aruco)
            #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        #moveit_commander.roscpp_shutdown()