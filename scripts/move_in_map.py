#!/usr/bin/python

import sys
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from move_to_pose_map import GoToPose
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi

# global odom_data

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

def odom_callback(data):  
    global odom_data
    odom_data = data.pose.pose.orientation
   
rospy.init_node('move_my_turtle')
turn_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
turn = Twist()



rate = rospy.Rate(10)
th=0.0
# (_, _, th_0) = euler_from_quaternion([odom_data.x, odom_data.y, odom_data.z, odom_data.w])
while abs(th) < pi/2:
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.sleep(0.1)
    (_, _, th_0) = euler_from_quaternion([odom_data.x, odom_data.y, odom_data.z, odom_data.w])
    th += th_0 
    turn.linear.x = 0.0
    turn.angular.z = 0.8
    turn_pub.publish(turn)
    rate.sleep()
    # (_, _, th) = euler_from_quaternion([odom_data.x, odom_data.y, odom_data.z, odom_data.w])
    print th

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

# moving.move_to_pose(0.0, 0.0, 180.0)
                # elif option == 3:
                #     moving.move_to_pose(4.56, -0.8, 135.0)
                # elif option == 4:
                #     moving.move_to_pose(4.56, -0.8, 135.0)
                # elif option == 5:
                #     moving.move_to_pose(4.56, -0.8, 135.0)
                # else:
                #     moving.move_to_pose(-0.846, 2.587, 135.0)

