#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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
        rospy.init_node('barcode_scan')  # Initialise rosnode

        # This will contain your image frame from camera
        self.img = np.empty([])

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()

        # sample time used for defining certain frequency of data input
        self.sample_time = 0.1

        self.box_switch=True
        self.attech_situation='False'
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
            cv2.imshow("Image window", self.img)
        except CvBridgeError as e:
            print(e)
            return

    def read_qr(self):
        print(type(self.img), int(self.img))
        # barcode = decode(self.img)
        print(self.img)
        # # (rows, cols, channels) = self.img.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(self.img, (50, 50), 10, 255)

        cv2.imshow("Image window", self.img)
        # cv2.imshow(self.img)
        # for code in barcode:
        #     print(code.data.decode('utf-8'))


if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(1/1)
    while not rospy.is_shutdown():
        image_proc_obj.read_qr()
        r.sleep()
    rospy.spin()
