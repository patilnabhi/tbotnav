#!/usr/bin/python

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

    cv2.imshow("Image", img)
    cv2.waitKey(3)


rospy.init_node('move_in_map')

# while not rospy.is_shutdown():
# moving = GoToPose()
#     option = 0
#     proceed = raw_input("Enter 'm' to move robot: ")
    
    
#     if(proceed == "m"):
#         proceed = "no"
#         while (proceed == "no"):
#             a = []
#             rospy.Subscriber("hand_img", Image, img_callback)
#             rospy.sleep(1)
#             # img = np.array(img, dtype=np.uint16)
            
#             print "Place your palm parallel to camera & align to center"
#             rospy.sleep(5)
#             for i in range(5):
#                 rospy.Subscriber("num_fingers", Int32, num_callback)
#                 rospy.sleep(1)
#                 a.append(num_fingers)
#                 print num_fingers
#             print "Please remove your hand"
#             rospy.sleep(2)
#             # cv2.destroyAllWindows()
#             # option = int(np.mean(a))
#             option = max(set(a), key=a.count)
#             print "Detected fingers = ", option
#             rospy.sleep(1)

#             proceed = raw_input("Do you want to proceed? (yes/no): ")

#             find_person = raw_input("Enter the name of person to move to: ")

#             if proceed == "yes":
#                 print "Robot starting to move..."

#                 if option == 2:
# moving.move_to_pose(1.7, -1.4, 0.0)
                # elif option == 3:
                #     moving.move_to_pose(4.56, -0.8, 135.0)
                # elif option == 4:
                #     moving.move_to_pose(4.56, -0.8, 135.0)
                # elif option == 5:
                #     moving.move_to_pose(4.56, -0.8, 135.0)
                # else:
                #     moving.move_to_pose(-0.846, 2.587, 135.0)

