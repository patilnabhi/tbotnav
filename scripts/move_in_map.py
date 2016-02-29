#!/usr/bin/python

import sys
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from move_to_pose_map import GoToPose
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi

class MoveTbot:
    def __init__(self):
        self.node_name = "move_tbot"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self._shutdown)

        self.bridge = CvBridge()
        self.turn = Twist()
        self.move = GoToPose()

        self.goal_x = []
        self.goal_y = []

        self.qr_data = AlvarMarkers()

        self.qr_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.qr_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        # self.num_fingers_sub = rospy.Subscriber('num_fingers', Int32, num_fingers_callback)
        # self.hand_img_sub = rospy.Subscriber('hand_img', Image, hand_img_callback)
        # self.face_img_sub = rospy.Subscriber('face_img', Image, face_img_callback)
        # self.face_name_sub = rospy.Subscriber('face_name', String, face_name_callback)

        self.turn_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print self.qr_data
        # self.tbot_routine()

    def qr_callback(self, data):
        # print data.markers[0].pose.pose.position.x
        # print data.markers[1].pose.pose.position.x
        self.qr_data = data.markers
        self.i=0
        
        # self.qr_position = data.markers.pose.pose.position

    def odom_callback(self, data):
        # print data
        self.odom_orient = data.pose.pose.orientation

    def num_fingers_callback(self, data):
        self.num_fingers = data.data

    def hand_img_callback(self, ros_image):
        try:
            self.hand_img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        cv2.imshow("Hand Image", self.hand_img)
        cv2.waitKey(3)

    def face_img_callback(self, ros_image):
        try:
            self.face_img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        cv2.imshow("Face Image", self.face_img)
        cv2.waitKey(3)

    def face_name_callback(self, data):
        self.face_name = data.data

    def rotate_tbot(self, th_0):        
        # th = th_0
        
        while self.i < 100:            
            # (_, _, th) = euler_from_quaternion([self.odom_orient.x, self.odom_orient.y, self.odom_orient.z, self.odom_orient.w])
            # if th_0 > np.pi/2:
            #     th_0 = np.pi - th_0
            #     if th < 0:
            #         th = -th
            #     else:
            #         th = np.pi - th
            
            self.turn.linear.x = 0.0
            self.turn.angular.z = 0.4

            self.turn_pub.publish(self.turn)
            # print th - th_0
            print self.i
            self.i += 1

            self.rate.sleep()

    def determine_goal_loc(self, proceed):
        if proceed == 'm':
            proceed = "n"
            while (proceed != "y"):
                a = []               

                print "Place your palm parallel to camera & align to center"
                rospy.sleep(5)
                for i in range(5):                    
                    a.append(self.num_fingers)
                    print self.num_fingers

                print "Please remove your hand"
                rospy.sleep(2)

                self.goal_loc = max(set(a), key=a.count)
                print "Station to move = ", self.goal_loc
                rospy.sleep(1)

                proceed = raw_input("Do you want to proceed? (y/n): ")

            if proceed == 'y':
                self.person = raw_input("Enter the name of person to move to: ")

    def find_qr_tags(self): # Function to store location (x, y) of detected qr tags in an array
        self.qr_pos = [[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]]
        for i in range(len(self.qr_id)):
            self.qr_pos[self.qr_id[i]] = [self.qr_position.x, self.qr_position.y]

    # def find_person(self):

    def tbot_routine(self):       
        # for i in range(4):
        (_, _, th_0) = euler_from_quaternion([self.odom_orient.x, self.odom_orient.y, self.odom_orient.z, self.odom_orient.w])
        self.rotate_tbot(th_0)

        # while not rospy.is_shutdown():
            
        #     proceed = raw_input("Press 'm' to move robot: ")
        #     self.determine_goal_loc(proceed)

        #     # Search for goal location
        #     for i in range(8):
        #         (_, _, th_0) = euler_from_quaternion([self.odom_orient.x, odom_orient.y, odom_orient.z, odom_orient.w])
        #         self.rotate_tbot(th_0)
                
        #         # QR detection
        #         found = 0       
        #         self.find_qr_tags()

        #         for j in range(len(self.qr_pos)):
        #             if self.qr_pos[j] == self.goal_loc:                    
        #                 found = 1
        #                 goal_idx = j
        #             else:
        #                 found = 0
        #         if found:
        #             break

        #     self.move.move_to_pose(self.qr_pos[goal_idx])

        #     self.find_person()
        


    def _shutdown(self):
        rospy.loginfo("Shutting down node...")

def test():
    print qr_data

if __name__ == '__main__':
    try:
        MoveTbot()
        # test()
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")


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
rospy.Subscriber('odom', Odometry, odom_callback)
rospy.sleep(0.1)
(_, _, th_0) = euler_from_quaternion([odom_data.x, odom_data.y, odom_data.z, odom_data.w])
th = th_0
while (th - th_0) < pi/2:
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.sleep(0.1)
    (_, _, th) = euler_from_quaternion([odom_data.x, odom_data.y, odom_data.z, odom_data.w])
    if th_0 > pi/2:
        th_0 = pi - th_0
        th = -th
    
    turn.linear.x = 0.0
    turn.angular.z = 0.8
    turn_pub.publish(turn)
    rate.sleep()
    # (_, _, th) = euler_from_quaternion([odom_data.x, odom_data.y, odom_data.z, odom_data.w])
    print (th - th_0)

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

