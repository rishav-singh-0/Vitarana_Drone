#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from sensor_msgs.msg import NavSatFix, LaserScan
import rospy
import math
import std_msgs.msg


class marker_detection():

    def __init__(self):
        rospy.init_node('marker_detect')

        self.img = np.empty([])
        # For conversion of rosmsg to cv2 image
        self.bridge = CvBridge()

        # image size is 400x400 pixel
        self.img_width = 400
        # camera's aperture
        self.hfov_rad = 1.3962634
        # focal lengath of camera lens
        self.focal_length = (self.img_width/2)/math.tan(self.hfov_rad/2)
        # initializetion for for detection error
        self.error = NavSatFix()
        # will save coordinates from the gps topic
        self.current_location = [0, 0, 0]
        # it will save the destination coordinates for the calculation purposes
        self.destination = [0, 0, 0]

        # sample time used for defining certain frequency of data input
        self.sample_time = 0.01
        # catech tje data from detectection
        self.logo_data = [0, 0, 0, 0]
        # data of bottom range
        self.obs_range_bottom = [0]
        self.logo_cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data/cascade.xml'))

        # Subscribe
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/destination_data', NavSatFix, self.destination_callback)

        # Publish
        self.marker_error = rospy.Publisher('/marker_error', NavSatFix, queue_size=1)

    # functions
    def destination_callback(self, msg):
        if(msg.latitude != 0):
            self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        if(msg.latitude != 0):
            self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_bottom_callback(self, msg):
        
        if(msg.ranges[0]>0.400000):
            self.obs_range_bottom = msg.ranges
            # print(self.obs_range_bottom[0])

    def image_callback(self, data):
        ''' Callback function of camera topic'''
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

    def detect_marker(self):
        '''Image QR-Code scanning and publishing algo'''
        if(self.img.size > 1):
            try:
                gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
                # image, reject levels level weights.
                logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
                if(len(logo) != 0 and logo[0][2]<80):
                    print("detected")
                    '''
                    - Providing error to the path_planner
                    - calculating necessary distance in meter
                    '''
                    row_x = -(200-(2*logo[0][0]+logo[0][2])/2)
                    row_y = (200-(2*logo[0][1]+logo[0][3])/2)

                    x = (row_x*(self.current_location[2]-self.destination[2]))/self.focal_length
                    y = (row_y*(self.current_location[2]-self.destination[2]))/self.focal_length
                    # x = (row_x*(self.obs_range_bottom[0]))/self.focal_length
                    # y = (row_y*(self.obs_range_bottom[0]))/self.focal_length

                    self.error.latitude = x
                    self.error.longitude = y

                    self.marker_error.publish(self.error)
                else:

                    self.error.latitude = 0
                    self.error.longitude = 0

                    self.marker_error.publish(self.error)

                # for (x, y, w, h) in logo:
                #     cv2.rectangle(self.img, (x, y),(x + w, y + h), (255, 255, 0), 2)
                # cv2.imshow("show", self.img)
                # cv2.waitKey(100)

            except ValueError:
                pass


if __name__ == '__main__':
    marker_detection_obj = marker_detection()
    r = rospy.Rate(1/marker_detection_obj.sample_time)
    while not rospy.is_shutdown():
        marker_detection_obj.detect_marker()
        r.sleep()
    rospy.spin()

