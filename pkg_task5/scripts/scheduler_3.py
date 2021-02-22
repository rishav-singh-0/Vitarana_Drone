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
import std_msgs.msg
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix


class Data_processing():

    def __init__(self):
        rospy.init_node('data_processing')

        self.delivery_grid = [18.9998102845, 72.000142461]
        self.return_grid = [18.9999367615, 72.000142461]
        self.diff_grid = [0.000013552, 0.000014245]
        #18.9993676146,71.9999999999,10.854960

        #*******************************************opt********************************#
        self.destination_list=[[18.9998102845, 72.000142461,16.757980880738472],
                               [18.99993956,72.0008224534,17.4189912398],
                               [(18.9999864489+0.000013552),(71.9999430161+0.000014245),8.44099749139],
                               [(18.9999864489+0.000013552),71.9999430161,8.44099749139]]
        self.cnt=0
        #*******************************************opt********************************#
        self.purpose=['DELIVERY','DELIVERY','RETURN','RETURN']


        self.sample_time = 0.01
        self.d_checkpoint = NavSatFix()

        # Publishing
        self.pub_checkpoint = rospy.Publisher('/box_checkpoint', NavSatFix, queue_size=1)

         #*******************************************opt********************************#
        #subscribing
        rospy.Subscriber('/next_destination_flag',Float32,self.next_destination_callback)

    #function for the subscription
    def next_destination_callback(self,msg):
        if(msg.data==1):
            if(self.cnt==3):
                self.cnt=3
            else:
                self.cnt+=1

    def read_data(self):

        # with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'manifest.csv'), 'r') as x:
        #     content = numpy.array(list(csv.reader(x)))
        #     self.box_type = content[:, 0]
        #     d_list = content[:, 1:]


        #*******************************************opt********************************#
        [self.d_checkpoint.latitude,self.d_checkpoint.longitude,self.d_checkpoint.altitude]=self.destination_list[self.cnt]
        self.d_checkpoint.header.frame_id=self.purpose[self.cnt]
        self.pub_checkpoint.publish(self.d_checkpoint)
        print(self.cnt)
        #*******************************************opt********************************#

if __name__ == "__main__":
    reader = Data_processing()
    rate = rospy.Rate(1/reader.sample_time)
    while not rospy.is_shutdown():
        reader.read_data()
        rate.sleep()
