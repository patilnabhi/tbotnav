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
        cv2.moveWindow("Depth Image", 25, 350)

        self.bridge = CvBridge()
        # self.hand_gestures = HandGestureRecognition()

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

    def process_depth_image(self, frame):
        outimg = frame
        np.clip(frame, 0, 2**10-1, frame)
        frame >>= 2
        frame = frame.astype(np.uint8)

        # num_fingers, outimg = self.hand_gestures.recognize(frame)        

        height, width = frame.shape[:2]
        cv2.circle(outimg, (width/2, height/2), 3, [255, 102, 0], 2)
        cv2.rectangle(outimg, (width/3, height/3), (width*2/3, height*2/3), [255, 102, 0], 2)

        return outimg

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