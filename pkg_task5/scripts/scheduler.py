#!/usr/bin/env python

'''
This python file runs a ROS-node 'data_processing' which takes care of the next destination to be reached
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /checkpoint             /marker_data

'''

import rospy
import math
import csv
import numpy
import os
from sensor_msgs.msg import NavSatFix


class Data_processing():

    def __init__(self):
        rospy.init_node('data_processing')

        self.delivery_grid = [18.9998102845, 72.000142461]
        self.return_grid = [18.9999367615, 72.000142461]
        self.diff_grid = [0.000013552, 0.000014245]

        self.sample_time = 1

        self.checkpoint = NavSatFix()

        # Publishing
        self.pub_checkpoint = rospy.Publisher('/box_checkpoint', NavSatFix, queue_size=1)

    def read_data(self):

        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'manifest.csv'), 'r') as x:
            content = numpy.array(list(csv.reader(x)))
            self.box_type = content[:, 0]
            d_list = content[:, 1:]


if __name__ == "__main__":
    reader = Data_processing()
    rate = rospy.Rate(1/reader.sample_time)
    while not rospy.is_shutdown():
        rate.sleep()
