#!/usr/bin/python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from rec_fingers import RecognizeNumFingers

class HandGestures:
    def __init__(self):
        self.node_name = "hand_gestures"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)

        self.cv_window_name = self.node_name
        cv2.namedWindow("Depth Image", 1)
        cv2.moveWindow("Depth Image", 20, 350)

        self.bridge = CvBridge()
        self.numFingers = RecognizeNumFingers() 

        self.depth_sub = rospy.Subscriber("/asus/depth/image_raw", Image, self.depth_callback)
        self.num_pub = rospy.Publisher('num_fingers', Int32, queue_size=10, latch=True)       
        self.img_pub = rospy.Publisher('hand_img', Image, queue_size=10)
        rospy.loginfo("Waiting for image topics...")        

    def depth_callback(self, ros_image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(ros_image)
        except CvBridgeError, e:
            print e

        inImgarr = np.array(inImg, dtype=np.uint16)
        # inImgarr = cv2.GaussianBlur(inImgarr, (3, 3), 0)
        # cv2.normalize(inImgarr, inImgarr, 0, 1, cv2.NORM_MINMAX) 
        
        self.outImg, self.num_fingers = self.process_depth_image(inImgarr) 
        # outImg = self.process_depth_image(inImgarr) 
        rate = rospy.Rate(10)        
        self.num_pub.publish(self.num_fingers)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.outImg, "bgr8"))
        rate.sleep()
                
        # cv2.imshow("Depth Image", self.outImg)
        # cv2.waitKey(3) 


    def process_depth_image(self, inImg):
        np.clip(inImg, 0, 1023, inImg)
        inImg >>= 2
        inImg = inImg.astype(np.uint8)

        outImg, num_fingers = self.numFingers.find(inImg)
        # outImg = self.numFingers.find(inImg)
       
        height, width = inImg.shape[:2]
        # print width
        cv2.circle(outImg, (width/2, height/2), 3, [0, 102, 255], 2)
        cv2.rectangle(outImg, (width/3, height/3), (width*2/3, height*2/3), [0, 102, 255], 2)
        cv2.putText(outImg, str(num_fingers), (width/3,height/4), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (122, 0, 255))
        
        return (outImg, num_fingers)
        # return outImg

    # def outImg_pub(self):
    # 	self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.outImg, "bgr8"))


    # def num_fingers_pub(self):
    #     rate = rospy.Rate(10)
    #     # while not rospy.is_shutdown():
    #     self.num_pub.publish(self.num_fingers)
    #     rate.sleep()

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        HandGestures()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."    
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)