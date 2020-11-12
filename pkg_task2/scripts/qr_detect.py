#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar.pyzbar import decode  # For decoding qrcode
import numpy as np
from sensor_msgs.msg import NavSatFix
import rospy


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

        # Publishing the scanned destination
        self.final_destination = rospy.Publisher('/final_setpoint', NavSatFix, queue_size=1)

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber('/edrone/camera/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        ''' Callback function of camera topic'''
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

    def read_qr(self):
        '''Image QR-Code scanning and publishing algo'''
        try:
            barcode = decode(self.img)
            data = [0, 0, 0]
            # used for loop to eleminate the possibility of multiple or null qrcode check
            for code in barcode:
                data = code.data.decode('utf-8')
                data = list(map(float, data.split(',')))
                # print(data)
            # cv2.imshow("show",self.img)
            # cv2.waitKey(100)

            # giving the scanned valut to publisher container
            self.destination.latitude = data[0]
            self.destination.longitude = data[1]
            self.destination.altitude = data[2]

            # Publishing the scanned data through /final_destination topic
            self.final_destination.publish(self.destination)

        except ValueError:
            pass


if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(1/image_proc_obj.sample_time)
    while not rospy.is_shutdown():
        image_proc_obj.read_qr()
        r.sleep()
    # cv2.destroyAllWindows()
    rospy.spin()
