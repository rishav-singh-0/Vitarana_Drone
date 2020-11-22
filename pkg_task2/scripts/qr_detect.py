#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar.pyzbar import decode  # For decoding qrcode
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import rospy


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('qrcode_scan')  # Initialise rosnode

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()

        # sample time used for defining certain frequency of data input
        self.sample_time = 0.1

        self.switch=True
        self.attech_situation='False'
        self.cross_co_ordinates=NavSatFix()
        self.data = [0, 0, 0]


        # Publishing the scanned destination
        self.final_destination = rospy.Publisher(
            '/final_setpoint', NavSatFix, queue_size=1)

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        #rospy.Subscriber('check_point_flag', Float32, self.detech_msg)

    def gripper_check_callback(self, state):
        self.attech_situation = state.data


    def detech_msg(self, msg):
        self.detech_req = msg.data
        print(self.detech_req)

    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            # print(e)
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
            print(data)

            if(self.data!=[0,0,0]):
                self.cross_co_ordinates.latitude=self.data[0]
                self.cross_co_ordinates.longitude=self.data[1]
                self.cross_co_ordinates.altitude=self.data[2]

            # self.destination.latitude = self.data[0]
            # self.destination.longitude = self.data[1]
            # self.destination.altitude = self.data[2]
            

            if(self.switch and self.attech_situation=='True'):
                self.destination.latitude = self.cross_co_ordinates.latitude
                self.destination.longitude = self.cross_co_ordinates.longitude
                self.destination.altitude = self.cross_co_ordinates.altitude
                #print(self.destination)
                self.switch=False
            elif(self.switch):
                self.destination.latitude = 19.0007046575 
                self.destination.longitude = 71.9998955286
                self.destination.altitude = 22.1599967919



            # Publishing the scanned data through /final_destination topic
            # self.final_destination.publish(self.destination)

            print(self.destination)
            self.final_destination.publish(self.destination)

        except ValueError:
            pass


if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
