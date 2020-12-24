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
        rospy.init_node('qrcode_scan')  # Initialise rosnode

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
        self.final_destination = rospy.Publisher('/final_setpoint', NavSatFix, queue_size=1)

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/edrone/gripper_check', String,self.gripper_check_callback)
        #rospy.Subscriber('check_point_flag', Float32, self.detech_msg)

    def gripper_check_callback(self, state):
        self.attech_situation = state.data

    def detech_msg(self, msg):
        self.detech_req = msg.data
        # print(self.detech_req)

    def image_callback(self, data):
        ''' Callback function of camera topic'''
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", self.img)
        except CvBridgeError as e:
            print(e)
            return

    def read_qr(self):
        '''Image QR-Code scanning and publishing algo'''
        try:
            barcode = decode(self.img)
            # used for loop to eleminate the possibility of multiple or null qrcode check
            for code in barcode:
                self.data = code.data.decode('utf-8')
                self.data = list(map(float, self.data.split(',')))
            # cv2.imshow("show",self.img)
            # cv2.waitKey(100)

            if(self.box_switch and self.attech_situation == 'True' and self.data != [0, 0, 0]):
                self.destination.latitude = self.data[0]
                self.destination.longitude = self.data[1]
                self.destination.altitude = self.data[2]
                # print(self.destination)
                self.box_switch = False

            if(self.box_switch):
                # Publishing the coordinates of box if box is not attached or data is not scanned
                self.destination.latitude = 19.0007046575
                self.destination.longitude = 71.9998955286
                self.destination.altitude = 22.1599967919

            # print(self.destination)
            self.final_destination.publish(self.destination)

        except ValueError, IndexError:
            pass

    def detect_marker(self):
        try:
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            # image, reject levels level weights.
            logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

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
        image_proc_obj.read_qr()
        r.sleep()
    # cv2.destroyAllWindows()
    rospy.spin()
