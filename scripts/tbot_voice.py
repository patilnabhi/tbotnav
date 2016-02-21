#!/usr/bin/python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class SayText:
    def __init__(self):
        self.sound_pub = rospy.Publisher("tbot_voice", SoundClient, queue_size=10)

    def say(self, text_msg):
        soundhandle = SoundClient()
        voice = 'voice_kal_diphone'
        soundhandle.say(text_msg, voice)
        rospy.sleep(1)
        