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

class Images:
    def __init__(self):
        
        # Bridge Instance
        self.bridge = CvBridge()
        global start
        # Get Compressed Image Topic from Subscriber
        self.image_sub = rospy.Subscriber("/d400/color/image_raw", Image, self.callback)

    
    def callback(self, data):
        global start 
        end = time.time()
        if end - start > 1:
            self.image_process(data)

    
    def image_process(self,data):
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
        pts2 = np.float32([[604-268/2,720-268/2*3/2],[604-268/2,720],[604+268/2,720-268/2*3/2],[604+268/2,720]])
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(cv_image,matrix,(width,height))
        
        
        # kmeans 
        # Z = dst.reshape((-1,3))
        # # convert to np.float32
        # Z = np.float32(Z)
        # # define criteria, number of clusters(K) and apply kmeans()
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        # K = 7
        # ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
        # # Now convert back into uint8, and make original image
        # center = np.uint8(center)
        # res = center[label.flatten()]
        # res2 = res.reshape((dst.shape))
        # print("time: ", time.time() - start_time)
        
        cv2.imshow('dst',dst)
        cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize
    
    camera = Images()
    rospy.init_node('Images', anonymous=False)
    rospy.spin()
    