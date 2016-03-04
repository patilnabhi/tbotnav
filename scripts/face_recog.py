#!/usr/bin/python

import rospy
import sys
import os
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class FaceRecognition:
    def __init__(self):
        self.node_name = "face_recog"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        
        self.size = 4
        self.fn_haar = 'haarcascade_frontalface_default.xml'
        self.haar_cascade = cv2.CascadeClassifier(self.fn_haar)
        self.fn_dir = 'face_data'
        self.model = cv2.createFisherFaceRecognizer()
        (self.im_width, self.im_height) = (112, 92)        

        rospy.loginfo("Loading data...")
        self.load_trained_data()

        self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.img_pub = rospy.Publisher('detect_face', Image, queue_size=10)
        rospy.loginfo("Detecting faces...")        

    def img_callback(self, image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError, e:
            print e
        
        inImgarr = np.array(inImg)

        
        try:
            self.outImg = self.process_image(inImgarr)         
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.outImg, "bgr8"))

            cv2.imshow("Recognise Face", self.outImg)
            cv2.waitKey(3)
        except:
            print "Failed! Ensure data is collected & trained..."

    def process_image(self, inImg):
        frame = cv2.flip(inImg,1,0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        
        mini = cv2.resize(gray, (gray.shape[1] / self.size, gray.shape[0] / self.size))        
        faces = self.haar_cascade.detectMultiScale(mini)
        for i in range(len(faces)):
            face_i = faces[i]
            (x, y, w, h) = [v * self.size for v in face_i]
            face = gray[y:y + h, x:x + w]
            face_resize = cv2.resize(face, (self.im_width, self.im_height))

            prediction = self.model.predict(face_resize)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            if prediction[1]<500:
                cv2.putText(frame, '%s - %.0f' % (self.names[prediction[0]],prediction[1]), (x-10, y-10), cv2.FONT_HERSHEY_PLAIN,1,(0, 255, 0))
            else:
                cv2.putText(frame, 'Unknown', (x-10, y-10), cv2.FONT_HERSHEY_PLAIN,1,(0, 255, 0))

        return frame

    def load_trained_data(self):
        (names, iden) = ({}, 0)
        for (subdirs, dirs, files) in os.walk(self.fn_dir):
            for subdir in dirs:
                names[iden] = subdir
                # subjectpath = os.path.join(self.fn_dir, subdir)
                # for filename in os.listdir(subjectpath):
                #     path = subjectpath + '/' + filename
                #     label = iden
                #     images.append(cv2.imread(path, 0))
                #     labels.append(int(label))
                iden += 1
        self.names = names
        

        # (images, labels) = [np.array(lis) for lis in [images, labels]]

        self.model.load('fisher_trained_data.xml')
        # self.model.train(images, labels)
        # self.model.save('training.xml')
        

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