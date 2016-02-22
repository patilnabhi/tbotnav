#!/usr/bin/python

import os
import rospy

rospy.init_node('test_voice')
# st = "Hello"
os.system("rosrun sound_play say.py 'Hi! Place your palm parallel to camera and align to center'")

