#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import cv2
from pyzbar.pyzbar import decode  # For decoding qrcode
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import rospy
import time


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('marker_detect')  # Initialise rosnode

        # This will contain your image frame from camera
        self.img = np.empty([])

        # initializing the final destination container
        self.destination = NavSatFix()

        # For conversion of rosmsg to cv2 image
        self.bridge = CvBridge()

        # sample time used for defining certain frequency of data input
        self.sample_time = 0.1

        self.box_switch = True
        self.attech_situation = 'False'
        self.data = [0, 0, 0]


        self.logo_cascade = cv2.CascadeClassifier('../data/cascade.xml')

        # Publishing the scanned destination
        # self.final_destination = rospy.Publisher('/final_setpoint', NavSatFix, queue_size=1)

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        # rospy.Subscriber('/edrone/gripper_check', String,self.gripper_check_callback)
        #rospy.Subscriber('check_point_flag', Float32, self.detech_msg)

    # def gripper_check_callback(self, state):
    #     self.attech_situation = state.data

    # def detech_msg(self, msg):
    #     self.detech_req = msg.data
    #     # print(self.detech_req)

    def image_callback(self, data):
        ''' Callback function of camera topic'''
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", self.img)
        except CvBridgeError as e:
            print(e)
            return

    def detect_marker(self):
        try:
            if(self.img.size > 1):
                gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

                # image, reject levels level weights.
                logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

                for (x, y, w, h) in logo:
                    cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
                plt.show()
        except :
            pass


if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(1/image_proc_obj.sample_time)
    while not rospy.is_shutdown():
        image_proc_obj.detect_marker()
        r.sleep()
    # cv2.destroyAllWindows()
    rospy.spin()
