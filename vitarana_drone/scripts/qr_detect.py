#!/usr/bin/env python

'''
This python file runs a ROS-node of name qrcode_scan which read data of qr_code and 
publishes decoded data to /final_checkpoint
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /final_setpoint         /edrone/camera/image_raw
                                /edrone/gripper_check
'''

# Team ID:          VD_983
# Theme:            Vitarana_Drone
# Author List:      Rishav Singh, Kashyap Joshi
# Filename:         qr_detect.py
# Functions:        read_qr, gripper_check_callback, detech_msg, image_callback
# Global variables: None

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
    '''
    Purpose:
    ---
    Take data from camera and scanning qr_code from above the box 
    Taking read data of qr_code and publishing decoded data to /final_checkpoint

    Input Arguments:
    ---
    None
    '''

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

        # Publishing the scanned destination
        self.final_destination = rospy.Publisher('/final_setpoint', NavSatFix, queue_size=1)

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)

    def gripper_check_callback(self, state):
        '''
        Purpose:
        ---
        To check if box is gripped or to be gripped

        Input Arguments:
        ---
        state :  [ String ]
            output of pid control system

        Returns:
        ---
        None

        Example call:
        ---
        gripper_check_callback(<state>)
        '''

        self.attech_situation = state.data

    def image_callback(self, data):
        '''
        Purpose:
        ---
        Reading data from camera of eDrone and storing in self.img variable

        Input Arguments:
        ---
        data :  [ Image ]
            Raw data from Camera Sensor

        Returns:
        ---
        None

        Example call:
        ---
        image_callback(<Image_data>)
        '''

        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", self.img)
        except CvBridgeError as e:
            print(e)
            return

    def read_qr(self):
        '''
        Purpose:
        ---
        Read the QR Code from Image data and decode it

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        read_qr()
        '''
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
            # Neglecting some false readings
            pass

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the imag_proc class and decode qr_code
if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(1/image_proc_obj.sample_time)
    while not rospy.is_shutdown():
        image_proc_obj.read_qr()
        r.sleep()
    # cv2.destroyAllWindows()
    rospy.spin()
