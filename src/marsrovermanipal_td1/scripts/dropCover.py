#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from gripperControl import *
from panel import *
import os
import tf
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

br = None
ls = None
aruco = {}
def callback(data): 
    print('in callback')
    # print(data)
    for i in range(len(data.transforms)):
        frame = data.transforms[i]
        # print(frame)
        if (frame.fiducial_id == 13):
            aruco[frame.fiducial_id] = frame.transform
            translation = (aruco[frame.fiducial_id].translation.x, aruco[frame.fiducial_id].translation.y, aruco[frame.fiducial_id].translation.z)
            rotation = (aruco[frame.fiducial_id].rotation.x, aruco[frame.fiducial_id].rotation.y, aruco[frame.fiducial_id].rotation.z, aruco[frame.fiducial_id].rotation.w)
            # transformer = tf.Transformer()
            br.sendTransform(translation, rotation, rospy.Time(0), "aruco" + str(frame.fiducial_id), "camera_link")
            ls.waitForTransform("fiducial_" + str(frame.fiducial_id),"base_link",rospy.Time(0), rospy.Duration(15.0))
            aruco[frame.fiducial_id] = ls.lookupTransform("base_link","fiducial_" + str(frame.fiducial_id), rospy.Time(0))
            br.sendTransform(aruco[frame.fiducial_id][0], aruco[frame.fiducial_id][1], rospy.Time(0), "Aruco_" + str(frame.fiducial_id), "base_link")
            rospy.sleep(1)
            rospy.set_param('tag13',aruco[frame.fiducial_id])
            os.system("rosnode kill aruco_detect")
            os.system("rosnode kill dropCover")
            
rospy.init_node("dropCover", anonymous=False)
group_name = "manipulator"
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander(group_name)
robot = moveit_commander.RobotCommander()
br = tf.TransformBroadcaster()
ls = tf.TransformListener()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
gripperPos("open")
rospy.sleep(2)
move_group.go([radians(150),radians(-61),radians(-118),radians(-77),radians(92),radians(58)])

rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback)
while True:
    try:
        rospy.spin()
    except KeyboardInterrupt:
        sys.exit()