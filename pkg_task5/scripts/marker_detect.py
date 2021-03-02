'''
#Team ID:            0983
#Theme:              VITARANA DRONE
#Author List:        Rishav Singh,Kashyap Joshi
#Filename:           marker_detect.py
#Functions:          destination_callback,gps_callback,range_finder_bottom_callback,image_callback,detect_marker
#Global Variables:   None
'''
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
        # For storing tha data of the image in form of numpy array
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

        # sample time used for defining certain frequency of data input(its samplining time in seconds)
        self.sample_time = 0.01
        # catech tje data from detectection
        self.logo_data = [0, 0, 0, 0]
        # data of bottom range
        self.obs_range_bottom = [0]
        # it will take processed data which was trained from command line with positive as well as negetive images. so we can locate the cross marker where its present.
        self.logo_cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data/cascade.xml'))

        # Subscribe
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/destination_data', NavSatFix, self.destination_callback)

        # Publish
        self.marker_error = rospy.Publisher('/marker_error', NavSatFix, queue_size=1)

    # functions
    def destination_callback(self, msg):
        '''
        Purpose:
        ---
        Its a callback function for subcribing destination so we can calculate altitude difference between drone and marker.
        publisher script is path_planner_test.py(it will contain approximate coordinates of destination)

        Input Argument:
        ---
        msg

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the "path_planner_beta" node.
        '''
        if(msg.latitude != 0):
            self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        '''
        Purpose:
        ---
        Its a callback function for subcribing gps(under /edrone) so we can aware with the drone's position.

        Input Argument:
        ---
        msg

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published on "/edrone/gps" topic.
        '''
        if(msg.latitude != 0):
            self.current_location = [msg.latitude, msg.longitude, msg.altitude]


    def image_callback(self, data):
        '''
        Purpose:
        ---
        Its a callback function for subcribing "/edrone/camera/image_raw" topic. Frome this we will get image data from the camera frame by frame. 

        Input Argument:
        ---
        data

        Returns:
        ---
        None(if there are no exceptions)

        Example call:
        ---
        Called automatically when appropreat data will being published on "/edrone/camera/image_raw" topic.
        '''

        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

    def detect_marker(self):
        '''
        Purpose:
        ---
        This function will process on the image data and remove fake pixel location where wrong marker like thing(ie: helipad) and will precess the pixel data and calculate coordinates of the
        cross marker in meters.
        by applying some oncepts of ray optics we can calculate the difference to cross marker only we need is focal length and pixel size of the image.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        Every time this function is calling for publishing the difference in distance from drone to cross marker in given frequency.
        '''

        if(self.img.size > 1):
            try:
                #converting RGB image form to GRAY image form
                gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
                # image, reject levels level weights.
                logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
                if(len(logo) != 0 and logo[0][2]<80):
                    print("detected")
                    '''
                    - Providing error to the path_planner
                    - calculating necessary distance in meter
                    '''
                    #row_ and row_y both will form central pixel of the detected image
                    row_x = -(200-(2*logo[0][0]+logo[0][2])/2)
                    row_y = (200-(2*logo[0][1]+logo[0][3])/2)

                    #x and y will contain the difference between drone and cross marker in meters
                    x = (row_x*(self.current_location[2]-self.destination[2]))/self.focal_length
                    y = (row_y*(self.current_location[2]-self.destination[2]))/self.focal_length
                    '''
                    below lines will publish the difference to the cross marker.
                    if no data is detected then it will publish 0 for both latitude and longitude errors.
                    '''
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
        r.sleep()                           #for working with perticular frequency otherwise it will take default frequency.
    rospy.spin()


