#!/usr/bin/python

import sys
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from move_to_pose import GoToPose
from std_msgs.msg import Int32, String
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi, radians

class MoveTbot:
    def __init__(self):
        self.node_name = "move_tbot"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self._shutdown)

        self.bridge = CvBridge()
        self.turn = Twist()
        self.move = GoToPose()

        self.qr_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.qr_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.num_fingers_sub = rospy.Subscriber('num_fingers', Int32, self.num_fingers_callback)
        self.hand_img_sub = rospy.Subscriber('hand_img', Image, self.hand_img_callback)
        # self.face_img_sub = rospy.Subscriber('face_img', Image, face_img_callback)
        # self.face_name_sub = rospy.Subscriber('face_name', String, face_name_callback)

        self.turn_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # self.run_tbot_routine()

    # def run_tbot_routine(self):
	        print "Gesture '5' to begin"
	        rospy.sleep(3)
	        # if self.detected_gesture == 5:
	        begin = 0
	        while begin != 5:
	            self.determine_gesture()
	            begin = self.detected_gesture

	        print "You gestured ", self.detected_gesture
	        rospy.sleep(3)
	        print "Gesture '2' or '3'"
	        rospy.sleep(1)
	        self.determine_gesture()

	        print "You gestured ", self.detected_gesture
	        rospy.sleep(1)

	        if self.detected_gesture == 2:  
	            rospy.loginfo("Entering station-finder mode...")
	            rospy.sleep(3)

	            rospy.loginfo("Rotating 360 deg...")
	            rospy.sleep(1)

	            self.rotate_tbot(360.0+120.0)
	            rospy.sleep(3)

	            print "Which station would you like me to move?"
	            rospy.sleep(1)
	            self.determine_gesture()

	            station_id = self.detected_gesture
	            print "You gestured ", self.detected_gesture
	            rospy.sleep(1)

	            station_loc = self.find_station(station_id)
	            print "Moving to station ", station_id

	            if station_loc[0]<0:
	                goal_x = station_loc[0] + 0.3
	            else:
	                goal_x = station_loc[0] - 0.3

	            if station_loc[1]<0:
	                goal_y = station_loc[1] + 0.3
	            else:
	                goal_y = station_loc[1] - 0.3

	            self.move_tbot(goal_x, goal_y)
	            
	            rospy.sleep(15)

    def qr_callback(self, data):
        self.qr_data = data.markers

    def odom_callback(self, data):
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

    def rotate_tbot(self, deg):
        num = int(deg/45.0)
        for i in range(num*10):
            self.turn.angular.z = radians(45.0)
            self.turn_pub.publish(self.turn)
            self.rate.sleep()

    def determine_gesture(self): 
        print "Place your palm infront of camera."
        cv2.imshow("Hand Image", self.hand_img)
        cv2.waitKey(3) 

        rospy.sleep(5)      
        a = []  
        rospy.loginfo("Detecting gesture...")
        for i in range(6):
            a.append(self.num_fingers)
            # print "Detected fingers: ", a[i]

        self.detected_gesture = max(set(a), key=a.count)

    def find_station(self, station_id):
        station_loc = []
        station_loc = self.qr_tag_loc(station_id)
        count=0
        while not station_loc:
            if count == 12:
                break
            self.rotate_tbot(90.0)
            station_loc = self.qr_tag_loc(station_id)
            rospy.sleep(3)
            count += 1
        return station_loc

    def qr_tag_loc(self, qr_id):
        if self.qr_data:
            for i in range(len(self.qr_data)):
                if self.qr_data[i].id == qr_id:
                    return [self.qr_data[i].pose.pose.position.x, self.qr_data[i].pose.pose.position.y]
        else:
            return []
        
    def move_tbot(self, goal_x, goal_y):
        self.move.move_to_pose(goal_x, goal_y)

    def _shutdown(self):
        rospy.loginfo("Shutting down node...")

if __name__ == '__main__':
    try:
        MoveTbot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")