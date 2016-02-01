#!/usr/bin/python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

class getImg:
    def __init__(self):
        self.node_name = "getImg"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)

        self.cv_window_name = self.node_name
        cv2.namedWindow("Depth Image", 1)
        cv2.moveWindow("Depth Image", 100, 350)

        self.bridge = CvBridge()

        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.depth_callback)
        
        rospy.loginfo("Waiting for image topics...")
        

    def depth_callback(self, ros_image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        inImgarr = np.array(inImg, dtype=np.uint8)
        outimg = self.process_depth_image(inImgarr)        
                
        cv2.imshow("Depth Image", outimg)
        cv2.waitKey(3)        

    def process_depth_image(self, inImg):
        # outImg, num_fingers = output_from_hand_recognization(inImg)
        outImg = inImg
        num_fingers = 3

        height, width = inImg.shape[:2]
        cv2.circle(outImg, (width/2, height/2), 3, [0, 0, 255], 2)
        cv2.rectangle(outImg, (width/3, height/3), (width*2/3, height*2/3), [255, 0, 0], 2)
        cv2.putText(outImg, str(num_fingers), (220,180), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0))
        
        return outImg

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        getImg()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."    
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)