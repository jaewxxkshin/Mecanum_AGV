#!/usr/bin/env python
   
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

start = 0

class k_means:
    def __init__(self,argv):
        # original_image = cv2.imread("images/island.jpeg")
        # self.img = cv2.cvtColor(original_image,cv2.COLOR_BGR2RGB)
        self.img = argv
        self.vectorized = self.img.reshape((-1,3))
        self.vectorized = np.float32(self.vectorized)

    
    def process(self, K):
        print (" Image Processing : K_means clustering ")
        criteria = (cv2.TERM_CRITERIA_EPS + 
			        cv2.TERM_CRITERIA_MAX_ITER,10,1.0)

        attempts = 10 
        ret,label,center = cv2.kmeans(self.vectorized,K,None,criteria,attempts,
                                        cv2.KMEANS_PP_CENTERS)

        center = np.uint8(center)
        res = center[label.flatten()]
        self.result_image = res.reshape((self.img.shape))

        after_kmeans = self.result_image
        cv2.imwrite('kmeans.png', after_kmeans)
        cv2.waitKey(1)

class Images:
    def __init__(self):
        
        # Bridge Instance
        self.bridge = CvBridge()
        global start 
        start = time.time()

        # Get Compressed Image Topic from Subscriber
        self.image_sub = rospy.Subscriber("/d400/color/image_raw", Image, self.callback)

    
    def callback(self, data):
        global start

        end = time.time()
        # print( " end time : ", end, "\t start time : ", start ,"\t sec : ", end - start)
        if end - start > 1:
            start = time.time()
            self.image_process(data)

    
    def image_process(self,data):
        # global start
        start_time = time.time()
        print (" Image Processing : Warping ")
        try:
            # Convert Image to OpenCV Format (Compressed Image Message to CV2)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        height, width = cv_image.shape[0], cv_image.shape[1]
        # print(height, width)   
        # print("test")
        # start_time = time.time()

        pts1 = np.float32([[443,478],[336,720],[855,474],[940,720]])
        pts2 = np.float32([[604-268/3,720-268/3*3/2],[604-268/3,720],[604+268/3,720-268/3*3/2],[604+268/3,720]])
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(cv_image,matrix,(width,height))
        
        
        # kmeans 
        # Z = dst.reshape((-1,3))
        # # convert to np.float32k_means
        
        k_m = k_means(dst)
        k_m.process(5)
        end_time = time.time()
        print("how long take : ",end_time - start_time)
        # cv2.imshow('dst',dst)
        # cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize
    
    camera = Images()
    rospy.init_node('Images', anonymous=False)
    rospy.spin()
    