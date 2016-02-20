#!/usr/bin/env python

import sys
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from move_to_pose_map import GoToPose
from std_msgs.msg import Int32

def num_callback(data):
    global num_fingers
    num_fingers = data.data

def img_callback(data):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print e

    # img = np.array(img)
    cv2.imshow("Image", img)
    cv2.waitKey(3)


rospy.init_node('move_in_map')

while not rospy.is_shutdown():
    proceed = raw_input("Type 'M' to move robot: ")
    
    if(proceed == "M"):
        proceed = "no"
        while (proceed == "no"):
            a = []
            rospy.Subscriber("outImg", Image, img_callback)
            rospy.sleep(1)
            # img = np.array(img, dtype=np.uint16)
            
            print "Place your palm parallel to camera & align to center"
            rospy.sleep(5)
            for i in range(5):
                rospy.Subscriber("num_fingers", Int32, num_callback)
                rospy.sleep(1)
                a.append(num_fingers)
                print num_fingers
            print "Please remove your hand"
            rospy.sleep(2)
            # option = int(np.mean(a))
            option = max(set(a), key=a.count)
            print "Detected fingers = ", option
            rospy.sleep(1)
            proceed = raw_input("Do you want to proceed? (yes/no): ")

            if proceed == "yes":
                print "Robot starting to move..."

                # option = 2
                # test = GoToPose()

                # if option == 1:
                #     test.move_to_pose(-9.798, -4.671, 135.0)

                # if option == 2:
                #     test.move_to_pose(-10.224, -0.311, 100.0)

                # if option == 3:
                #     test.move_to_pose(3.618, 0.727, 85.0)
