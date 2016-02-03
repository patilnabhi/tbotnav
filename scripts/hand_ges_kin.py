#!/usr/bin/python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from rec_fingers import RecognizeNumFingers

class getImg:
    def __init__(self):
        self.node_name = "getImg"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)

        self.cv_window_name = self.node_name
        cv2.namedWindow("Depth Image", 1)
        cv2.moveWindow("Depth Image", 20, 350)

        self.bridge = CvBridge()
        self.numFingers = RecognizeNumFingers() 

        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.depth_callback)
        
        rospy.loginfo("Waiting for image topics...")
        

    def depth_callback(self, ros_image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="mono16")
        except CvBridgeError, e:
            print e

        inImgarr = np.array(inImg, dtype=np.uint8)
        outimg = self.process_depth_image(inImgarr)        
                
        cv2.imshow("Depth Image", outimg)
        cv2.waitKey(3)        

    def process_depth_image(self, inImg):
        # np.clip(inImg, 0, 2**10-1, inImg)
        # inImg >>= 2
        # inImg = inImg.astype(np.uint8)

        outImg, num_fingers = self.numFingers.find(inImg)

        height, width = inImg.shape[:2]
        cv2.circle(outImg, (width/2, height/2), 3, [0, 102, 255], 2)
        cv2.rectangle(outImg, (width/3, height/3), (width*2/3, height*2/3), [0, 102, 255], 2)
        cv2.putText(outImg, str(num_fingers), (width/2,height/4), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (122, 0, 255))
        
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