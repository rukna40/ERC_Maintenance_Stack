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
    br = None
    ls = None
    pose = {1:([],[]),2:([],[]),3:([],[]),4:([],[]),5:([],[]),6:([],[]),7:([],[]),8:([],[]),9:([],[]),10:([],[]),11:([],[]),12:([],[]),13:([],[]),14:([],[])}

    def avgTransform(self, transform, newTransform):
        if transform==([],[]):
            transform=newTransform
        else:
            transform[0][0] = (transform[0][0] + newTransform[0][0])/2
            transform[0][1] = (transform[0][1] + newTransform[0][1])/2
            transform[0][2] = (transform[0][2] + newTransform[0][2])/2
            transform[1][0] = (transform[1][0] + newTransform[1][0])/2
            transform[1][1] = (transform[1][1]+ newTransform[1][1])/2
            transform[1][2] = (transform[1][2] + newTransform[1][2])/2
            transform[1][3] = (transform[1][3] + newTransform[1][3])/2
        return transform
        # sumtx=0
        # sumty=0
        # sumtz=0
        # sumrx=0
        # sumry=0
        # sumrz=0
        # sumrw=0
        # for i in pos[id]:
        #     sumtx+=i[0][0]
        #     sumty+=i[0][1]
        #     sumtz+=i[0][2]
        #     sumrx+=i[0][3]
        #     sumry+=i[1][0]
        #     sumrz+=i[1][1]
        #     sumrw+=i[1][2]

        # self.aruco[id].translation.x = sumtx/len(pos[id])
        # self.aruco[id].translation.y = sumty/len(pos[id])
        # self.aruco[id].translation.z = sumtz/len(pos[id])
        # self.aruco[id].rotation.x = sumrx/len(pos[id])       
        # self.aruco[id].rotation.y = sumry/len(pos[id])
        # self.aruco[id].rotation.z = sumrz/len(pos[id])
        # self.aruco[id].rotation.w = sumrw/len(pos[id])   
        
    def getTranslationAndRotation(transform): 
        return transform[0], transform[1]

    def callback(self,data):
        #print('in callback')
        now = rospy.Time.now()
        for i in range(len(data.transforms)):
            frame = data.transforms[i]
            #print(frame)
            self.aruco[frame.fiducial_id] = frame.transform
            translation = (self.aruco[frame.fiducial_id].translation.x, self.aruco[frame.fiducial_id].translation.y, self.aruco[frame.fiducial_id].translation.z)
            rotation = (self.aruco[frame.fiducial_id].rotation.x, self.aruco[frame.fiducial_id].rotation.y, self.aruco[frame.fiducial_id].rotation.z, self.aruco[frame.fiducial_id].rotation.w)
            transformer = tf.Transformer()
            self.br.sendTransform(translation, rotation, rospy.Time(0), "aruco" + str(frame.fiducial_id), "camera_link")
            self.ls.waitForTransform("fiducial_" + str(frame.fiducial_id),"base_link",rospy.Time(0), rospy.Duration(15.0))
            new_pose = self.ls.lookupTransform("base_link","fiducial_" + str(frame.fiducial_id), rospy.Time(0))
            self.br.sendTransform(new_pose[0], new_pose[1], rospy.Time(0), "Aruco" + str(frame.fiducial_id), "base_link")
            self.aruco[frame.fiducial_id]=self.avgTransform(self.pose[frame.fiducial_id],new_pose)
            self.pose[frame.fiducial_id]=self.aruco[frame.fiducial_id]
            self.br.sendTransform(new_pose[0], new_pose[1], rospy.Time(0), "Aruco" + str(frame.fiducial_id), "base_link")
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