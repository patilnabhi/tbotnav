#!/usr/bin/python

import rospy
import sys
import os
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class FaceRecognition:
    def __init__(self):
        self.node_name = "face_recog"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        
        self.size = 4
        face_haar = 'haarcascade_frontalface_default.xml'
        self.haar_cascade = cv2.CascadeClassifier(face_haar)
        self.face_dir = 'face_data'
        self.model = cv2.createFisherFaceRecognizer()
        (self.im_width, self.im_height) = (112, 92)        

        rospy.loginfo("Loading data...")
        self.load_trained_data()

        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.img_pub = rospy.Publisher('face_img', Image, queue_size=10)
        self.name_pub = rospy.Publisher('face_name', String, queue_size=10)
        rospy.loginfo("Detecting faces...")        

    def img_callback(self, image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError, e:
            print e 
                   
        inImgarr = np.array(inImg)
        try:
            self.outImg, self.face_names = self.process_image(inImgarr) 
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.outImg, "bgr8"))
            self.name_pub.publish(self.face_names)

            cv2.imshow("Recognise Face", self.outImg)
            cv2.waitKey(3)

        except:
            print "Failed! Ensure data is collected & trained..."

    def process_image(self, inImg):
        frame = cv2.flip(inImg,1,0)
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        
        cropped = cv2.resize(grayImg, (grayImg.shape[1] / self.size, grayImg.shape[0] / self.size))        
        faces = self.haar_cascade.detectMultiScale(cropped)
        persons = []
        for i in range(len(faces)):
            face_i = faces[i]
            x = face_i[0] * self.size
            y = face_i[1] * self.size
            w = face_i[2] * self.size
            h = face_i[3] * self.size
            face = grayImg[y:y + h, x:x + w]
            face_resize = cv2.resize(face, (self.im_width, self.im_height))
            confidence = self.model.predict(face_resize)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            if confidence[1]<1000:
                person = self.names[confidence[0]]
                cv2.putText(frame, '%s - %.0f' % (person, confidence[1]), (x-10, y-10), cv2.FONT_HERSHEY_PLAIN,1,(0, 255, 0))
            else:
                person = 'Unknown'
                cv2.putText(frame, person, (x-10, y-10), cv2.FONT_HERSHEY_PLAIN,1,(0, 255, 0))
            persons.append(person)
        return (frame, persons)

    def load_trained_data(self):
        names = {}
        index = 0
        for (subdirs, dirs, files) in os.walk(self.face_dir):
            for subdir in dirs:
                names[index] = subdir
                index += 1
        self.names = names 
        self.model.load('fisher_trained_data.xml')

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        FaceRecognition()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."    
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)