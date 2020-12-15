#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
import rospy
import time


class marker_detection():

    def __init__(self):
        rospy.init_node('marker_detect') 

        self.img = np.empty([])
        # For conversion of rosmsg to cv2 image
        self.bridge = CvBridge()

        # sample time used for defining certain frequency of data input
        self.sample_time = 0.1

        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        ''' Callback function of camera topic'''
        try:
            # Converting the image to OpenCV standard image
            #print(type(data))
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("show",self.img)
            #cv2.waitKey(100)
        except CvBridgeError as e:
            # print(e)
            return

    def detect_marker(self):
        '''Image QR-Code scanning and publishing algo'''
        if(self.img.size>1):
            try:
                logo_cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname(os.path.realpath(__file__)),'../data/cascade.xml'))

            #print(self.img.size)
            #print(type(self.img[1][1][1)

            # img = cv2.imread('test_2.png')  # Source image
            #self.img=self.img.astype(np.uint8)
                gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            # image, reject levels level weights.
                logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
                # print(logo[0])

                for (x, y, w, h) in logo:
                    cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                # plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
                # plt.show()
                cv2.imshow("show",self.img)
                cv2.waitKey(5)
            except ValueError, IndexError:
                pass


if __name__ == '__main__':
    marker_detection_obj = marker_detection()
    r = rospy.Rate(1/marker_detection_obj.sample_time)
    while not rospy.is_shutdown():

        marker_detection_obj.detect_marker()
        r.sleep()
    rospy.spin()
