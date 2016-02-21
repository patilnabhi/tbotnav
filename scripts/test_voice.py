#!/usr/bin/python

import tbot_voice
import rospy


rospy.init_node('test_voice')

vocalizer = tbot_voice.SayText()
mesg = "Hello!"

vocalizer.say(mesg)
rospy.sleep(1)