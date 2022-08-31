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

src = cv2.imread("contour.png")
src_bin = np.zeros((3,1280,720))

count_frame = 0
class k_means:
    def __init__(self,argv):
        self.img = argv
        self.vectorized = self.img.reshape((-1,3))
        self.vectorized = np.float32(self.vectorized)

    
    def process(self, K):
        criteria = (cv2.TERM_CRITERIA_EPS + 
			        cv2.TERM_CRITERIA_MAX_ITER,10,1.0)

        attempts = 10 
        ret,label,center = cv2.kmeans(self.vectorized,K,None,criteria,attempts,
                                        cv2.KMEANS_PP_CENTERS)

        center = np.uint8(center)
        res = center[label.flatten()]
        self.result_image = res.reshape((self.img.shape))
              
        # figure_size = 15
        # plt.figure(figsize=(figure_size,figure_size))
        # plt.subplot(1,2,1),plt.imshow(self.img)
        # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        # plt.subplot(1,2,2),plt.imshow(self.result_image)
        # plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
        # plt.show() 
        after_kmeans = self.result_image
        cv2.imwrite('kmeans.png', after_kmeans)
        cv2.waitKey(1)
    def test_jw(self):
        i = 1
        i += 1
        print(i)

class Images:
    def __init__(self):
        
        # Bridge Instance
        self.bridge = CvBridge()
        
        # Get Compressed Image Topic from Subscriber
        self.image_sub = rospy.Subscriber("/d400/color/image_raw", Image, self.callback)

    
    def callback(self, data):
        try:
            # Convert Image to OpenCV Format (Compressed Image Message to CV2)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        height, width = cv_image.shape[0], cv_image.shape[1]

        
        start_time = time.time()

        pts1 = np.float32([[443,478],[336,720],[855,474],[940,720]])
        pts2 = np.float32([[604-268/2,720-268/2*3/2],[604-268/2,720],[604+268/2,720-268/2*3/2],[604+268/2,720]])
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        global src_bin
        src_bin = cv2.warpPerspective(cv_image,matrix,(width,height))
        global count_frame
        count_frame += 1
        
        # #print(height, width)    
        # #cv2.imshow("test", dst)
        # #cv2.waitKey(1)
        
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

        # cv2.imshow('res2',res2)
        # cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize
    camera = Images()
    rospy.init_node('Images', anonymous=False)
    print(count_frame)
    # if count_frame % 6 == 0:
    #     k_m = k_means(src_bin)
    #     k_m.process(5)
    #     k_m.test_jw()
    rospy.spin()