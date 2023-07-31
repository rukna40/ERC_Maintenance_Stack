#!/usr/bin/env python3
import sys
import rospy
import tf
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import geometry_msgs.msg
import moveit_commander 
import random
import copy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from math import pi
#from buttons import *
import moveit_msgs.msg
class arucoPos:
    aruco = {}
    br = None
    ls = None

    def avgTransform(transform, newTransform):
        transform.translation.x = (transform.translation.x + newTransform.translation.x)/2
        transform.translation.y = (transform.translation.y + newTransform.translation.y)/2
        transform.translation.z = (transform.translation.z + newTransform.translation.z)/2
        transform.rotation.x = (transform.rotation.x + newTransform.rotation.x)/2
        transform.rotation.y = (transform.rotation.y + newTransform.rotation.y)/2
        transform.rotation.z = (transform.rotation.z + newTransform.rotation.z)/2
        transform.rotation.w = (transform.rotation.w + newTransform.rotation.w)/2
        return transform
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
            self.aruco[frame.fiducial_id] = self.ls.lookupTransform("base_link","fiducial_" + str(frame.fiducial_id), rospy.Time(0))
            self.br.sendTransform(self.aruco[frame.fiducial_id][0], self.aruco[frame.fiducial_id][1], rospy.Time(0), "Aruco_" + str(frame.fiducial_id), "base_link")
                #br.sendTransform(trans, rot, now, "camera_link", "base_link")

    def exec(self):
        print('in exec')
        robot = moveit_commander.RobotCommander()
        self.br = tf.TransformBroadcaster()
        self.ls = tf.TransformListener()
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray, self.callback )
        while not rospy.is_shutdown() and len(self.aruco)<14:
            rospy.spin()
            print(self.aruco)
            #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        #moveit_commander.roscpp_shutdown()