from shutil import move
import rospy
from geometry_msgs.msg import Pose
from math import radians
import copy
from gripperControl import *
from std_msgs.msg import Bool

flag = False

def callback(data):
    global flag
    flag = data.data
    if flag:
        print("Button has been pressed")
    else:
        print("Button has been released")

def extend_position(move_group, max_steps):
    current_step = 0
    currentPose = move_group.get_current_pose().pose
    while not flag and current_step < max_steps:
        currentPose.position.x += 0.001
        waypoints = []
        waypoints.append(copy.deepcopy(currentPose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        move_group.execute(plan, wait=True)
        current_step += 1
    rospy.sleep(1)

def press(id, move_group):
    global flag
    move_group.go([radians(0), radians(-120), radians(100), radians(20), radians(90), radians(-90)])
    gripperPos("close")
    move_group.set_pose_reference_frame('base_link')
    rospy.Subscriber("/button"+str(id), Bool, callback)

    if rospy.has_param('tag'+str(id)):
        pose = rospy.get_param('tag'+str(id))
        position = pose[0]
        button = Pose()
        currentPose = move_group.get_current_pose().pose
        waypoints = []
        minus = 0.114
        if id <= 3: #6 for real case
            minus = 0.126
        currentPose.position.x = position[0] - minus
        currentPose.position.z = position[2] - 0.055
        waypoints.append(copy.deepcopy(currentPose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        move_group.execute(plan, wait=True)

        #if id >= 0:
        currentPose.position.y = position[1]
        waypoints = []
        waypoints.append(copy.deepcopy(currentPose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        move_group.execute(plan, wait=True)
        max_steps = 22
        
        # Extend the gripper
        extend_position(move_group, max_steps)

        #comeback step
        currentPose.position.x -= 0.03
        waypoints = []
        waypoints.append(copy.deepcopy(currentPose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        move_group.execute(plan, wait=True)

        if id in (2, 3, 4):  # for the real case put 4-9
            move_group.go([radians(-70), radians(-130), radians(130), radians(0), radians(20), radians(-90)])
        else:
            move_group.go([radians(0), radians(-120), radians(100), radians(20), radians(90), radians(-90)])
    else:
        rospy.logerr("Parameter 'tag%d' not found." % id)
    flag = False
