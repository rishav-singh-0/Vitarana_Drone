#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import rospy


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('barcode_scan')  # Initialise rosnode
        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)
        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()

    # Callback function of camera topic

    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

    def read_qr(self):
        barcode = decode(self.img)
        for code in barcode:
            print(code.data.decode('utf-8'))


if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
