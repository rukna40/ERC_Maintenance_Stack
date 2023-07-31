#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Krzysztof Stezala
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# ROS node for checking the detected ArUco positions in the simulation

import rospy
import yaml
import numpy as np
import roslib

from erc_aruco_msg.srv import ErcArucoRequest, ErcArucoResponse, ErcAruco

class ErcArUcoChecker:
    def __init__(self):
        rospy.init_node("erc_aruco_checker",anonymous=True)
        pkg_dir = roslib.packages.get_pkg_dir("erc_aruco_checker")
        sim = rospy.get_param('~sim')
        self.tolerance = rospy.get_param('~tolerance')

        if sim:
            rospy.loginfo("Running in simulation")
            config_file = pkg_dir + "/config/sim_config.yaml"
            with open(config_file, 'r') as file_yaml:
                self.gt_config = yaml.safe_load(file_yaml)
        else:
            rospy.loginfo("Running in real")
            config_file = pkg_dir + "/config/real_config.yaml"
            with open(config_file, 'r') as file_yaml:
                self.gt_config = yaml.safe_load(file_yaml)

        self.check_service = rospy.Service('erc_aruco_score',ErcAruco,self.handle_score)
        rospy.loginfo("Ready to score")
        rospy.spin()

    def handle_score(self,req):
        points = 0.0
        for a in dir(req):
            if a.startswith('tag'):
                tag_name = "req." + a
                user_position = np.array(eval(tag_name))
                gt_position = np.array(self.gt_config[a])
                print(user_position, gt_position)
                dist = np.linalg.norm(user_position-gt_position)
                if dist <= self.tolerance and self.gt_config[a] != [0,0,0]:
                    points = points + 1.0

        return points

if __name__ == '__main__':
    try:
        c = ErcArUcoChecker()
    except KeyboardInterrupt:
        print("end")