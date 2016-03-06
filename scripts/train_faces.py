#!/usr/bin/python

import rospy
import sys
import os
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class TrainFisherFaces:
    def __init__(self):
        self.node_name = "train_faces"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        
        self.size = 4
        face_haar = 'haarcascade_frontalface_default.xml'
        self.haar_cascade = cv2.CascadeClassifier(face_haar)
        self.face_dir = 'face_data'
        self.face_name = sys.argv[1]
        self.path = os.path.join(self.face_dir, self.face_name)
        self.model = cv2.createFisherFaceRecognizer()
        self.cp_rate = 15

        if not os.path.isdir(self.path):
            os.mkdir(self.path)

        self.count = 0    

        self.train_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        # self.train_img_pub = rospy.Publisher('train_face', Image, queue_size=10)
        rospy.loginfo("Capturing data...")    

    def img_callback(self, image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError, e:
            print e    

        inImgarr = np.array(inImg)
        self.outImg = self.process_image(inImgarr)
        
        # self.train_img_pub.publish(self.bridge.cv2_to_imgmsg(self.outImg, "bgr8"))    

        # cv2.namedWindow("Capture Face")
        cv2.imshow('Capture Face', self.outImg)
        cv2.waitKey(3)

        if self.count == 10*self.cp_rate:
            rospy.loginfo("Data Captured!")
            # rospy.loginfo("Training Data...")
            # self.fisher_train_data()
            rospy.signal_shutdown('done')       

    def process_image(self, inImg):
        (self.frame_width, self.frame_height) = (112, 92)        
        frame = cv2.flip(inImg,1,0)
        grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cropped = cv2.resize(grayImg, (grayImg.shape[1] / self.size, grayImg.shape[0] / self.size))        
        faces = self.haar_cascade.detectMultiScale(cropped)
        faces = sorted(faces, key=lambda x: x[3])  
        if faces:
            face_i = faces[0] 
            x = face_i[0] * self.size
            y = face_i[1] * self.size
            w = face_i[2] * self.size
            h = face_i[3] * self.size
            face = grayImg[y:y + h, x:x + w]
            face_resize = cv2.resize(face, (self.frame_width, self.frame_height))
            img_no = sorted([int(fn[:fn.find('.')]) for fn in os.listdir(self.path) if fn[0]!='.' ]+[0])[-1] + 1
            if self.count % self.cp_rate == 0:
                cv2.imwrite('%s/%s.png' % (self.path, img_no), face_resize)
                print "Captured Img: ", self.count/self.cp_rate + 1
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(frame, self.face_name, (x - 10, y - 10), cv2.FONT_HERSHEY_PLAIN, 1,(0, 255, 0))            
            self.count += 1 
        return frame

    def fisher_train_data(self):        
        try:
            imgs = []
            tags = []
            index = 0

            for (subdirs, dirs, files) in os.walk(self.face_dir):
                for subdir in dirs:
                    img_path = os.path.join(self.face_dir, subdir)
                    for fn in os.listdir(img_path):
                        path = img_path + '/' + fn
                        tag = index
                        imgs.append(cv2.imread(path, 0))
                        tags.append(int(tag))
                    index += 1
            (imgs, tags) = [np.array(item) for item in [imgs, tags]]

            self.model.train(imgs, tags)
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