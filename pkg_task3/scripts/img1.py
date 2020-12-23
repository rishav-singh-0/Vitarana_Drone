#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import NavSatFix, LaserScan
import rospy
import time
import math


class marker_detection():

    def __init__(self):
        rospy.init_node('marker_detect') 

        self.img = np.empty([])
        # For conversion of rosmsg to cv2 image
        self.bridge = CvBridge()

        self.img_width = 400
        self.hfov_rad = 1.3962634
        self.focal_length = (self.img_width/2)/math.tan(self.hfov_rad/2)
        self.error=NavSatFix()

        # sample time used for defining certain frequency of data input
        self.sample_time = 0.1
        self.logo_data=[0,0,0,0]
        self.obs_range_bottom=[0]

        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)


        self.marker_error = rospy.Publisher('/marker_error', NavSatFix, queue_size=1)




    def range_finder_bottom_callback(self, msg):
        self.obs_range_bottom = msg.ranges


    def image_callback(self, data):
        ''' Callback function of camera topic'''
        try:
            # Converting the image to OpenCV standard image
            #print(type(data))
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv2.imshow("show",self.img)
            #cv2.waitKey(100)
        except CvBridgeError as e:
            # print(e)
            return

    def detect_marker(self):
        '''Image QR-Code scanning and publishing algo'''
        if(self.img.size>1):
            try:
                logo_cascade = cv2.CascadeClassifier('/home/kashyap/catkin_ws/src/pkg_task3/data/cascade.xml')
            #print(self.img.size)
            #print(type(self.img[1][1][1)

            # img = cv2.imread('test_2.png')  # Source image
            #self.img=self.img.astype(np.uint8)
                gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            # image, reject levels level weights.
                logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
                # print(logo[0])
                if(len(logo)!=0):
                    # print("detected")
                    print(logo)
                    # for i in range(4):
                    if(logo[0][1]>200):

                        row_y=row_x=-(200-(2*logo[0][0]+logo[0][2])/2)
                    else:
                        if(logo[0][0]<200):
                            row_x=-(200-(2*logo[0][0]+logo[0][2])/2)
                            row_y=(200-(2*logo[0][1]+logo[0][3])/2)
                        else:
                            row_x=-(200-(2*logo[0][0]+logo[0][2])/2)
                            row_y=200-(2*logo[0][1]+logo[0][3])/2


                        


                    # row_y=(200-(2*logo[0][1]+logo[0][3])/2)

                        
                        
                    
                    x=(row_x*self.obs_range_bottom[0])/self.focal_length
                    y=(row_y*self.obs_range_bottom[0])/self.focal_length
                    # (((self.logo_data[1]+self.logo_data[3])/2)*self.obs_range_bottom[0])/self.focal_length
                    self.error.latitude=x
                    self.error.longitude=y
                    print("hello")
                    # print(self.logo_data)
                    self.marker_error.publish(self.error)

                for (x, y, w, h) in logo:

                    cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                # plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
                # plt.show()
                cv2.imshow("show",self.img)
                cv2.waitKey(100)
            except ValueError, IndexError:
                pass


if __name__ == '__main__':
    marker_detection_obj = marker_detection()
    r = rospy.Rate(1/marker_detection_obj.sample_time)
    while not rospy.is_shutdown():

        marker_detection_obj.detect_marker()
        r.sleep()
    rospy.spin()
