#!/usr/bin/python

import rospy
import sys
import os
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class TrainFisherFaces:
    def __init__(self):
        self.node_name = "get_faces"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        
        self.size = 4
        self.fn_haar = 'haarcascade_frontalface_default.xml'
        self.haar_cascade = cv2.CascadeClassifier(self.fn_haar)
        self.fn_dir = 'face_data'
        self.fn_name = sys.argv[1]
        self.path = os.path.join(self.fn_dir, self.fn_name)
        self.model = cv2.createFisherFaceRecognizer()

        if not os.path.isdir(self.path):
            os.mkdir(self.path)

        self.count = 0    

        self.train_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.train_img_pub = rospy.Publisher('train_face', Image, queue_size=10)
        rospy.loginfo("Capturing data...")    

    def img_callback(self, image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError, e:
            print e
        
        inImgarr = np.array(inImg)

        self.outImg = self.process_image(inImgarr)
        self.train_img_pub.publish(self.bridge.cv2_to_imgmsg(self.outImg, "bgr8"))
    
        cv2.namedWindow("Capture Face")
        cv2.imshow('Capture Face', self.outImg)
        cv2.waitKey(3)

        if self.count == 10*5:
            rospy.loginfo("Data Captured!")
            rospy.loginfo("Training Data...")
            self.fisher_train_data()

            rospy.signal_shutdown('done')       

    def process_image(self, inImg):
        (self.frame_width, self.frame_height) = (112, 92)     
        
        frame = cv2.flip(inImg,1,0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        
        mini = cv2.resize(gray, (gray.shape[1] / self.size, gray.shape[0] / self.size))
        
        faces = self.haar_cascade.detectMultiScale(mini)
        faces = sorted(faces, key=lambda x: x[3])
        
        if faces:
            face_i = faces[0]
            (x, y, w, h) = [v * self.size for v in face_i]
            face = gray[y:y + h, x:x + w]
            face_resize = cv2.resize(face, (self.frame_width, self.frame_height))
            pin=sorted([int(n[:n.find('.')]) for n in os.listdir(self.path) if n[0]!='.' ]+[0])[-1] + 1
            if self.count % 5 == 0:
                cv2.imwrite('%s/%s.png' % (self.path, pin), face_resize)
                print "Captured Img: ", self.count/5 + 1
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(frame, self.fn_name, (x - 10, y - 10), cv2.FONT_HERSHEY_PLAIN, 1,(0, 255, 0))            
            self.count += 1     

        return frame

    def fisher_train_data(self):        
        try:
            (images, labels, iden) = ([], [], 0)
            for (subdirs, dirs, files) in os.walk(self.fn_dir):
                for subdir in dirs:
                    # names[iden] = subdir
                    subjectpath = os.path.join(self.fn_dir, subdir)
                    for filename in os.listdir(subjectpath):
                        path = subjectpath + '/' + filename
                        label = iden
                        images.append(cv2.imread(path, 0))
                        labels.append(int(label))
                    iden += 1

            (images, labels) = [np.array(lis) for lis in [images, labels]]

            self.model.train(images, labels)
            self.model.save('fisher_trained_data.xml')
            rospy.loginfo("Training completed successfully.")
        except:
            print "Training failed! Ensure that enough data is collected."

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        TrainFisherFaces()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."    
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)