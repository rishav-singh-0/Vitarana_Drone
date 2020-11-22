#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix, LaserScan, Imu

class PathPlanner():

    def __init__(self):
        rospy.init_node('path_planner_beta')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]
        self.destination_xy = [0, 0]

        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]
        self.current_location_xy = [0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()

        # 
        self.obs_range_top = []
        self.obs_range_bottom = []

        # self.yaw_error = 0
        self.diff_xy = [0, 0]
        self.distance_xy = 0

        self.movement_in_1D = 0
        self.movement_in_plane = [0, 0]     # [x, y]
        self.obs_closest_range = 10          # closest distance of obstacle (in meters)

        self.sample_time = 0.01

        # Publisher
        self.pub_checkpoint = rospy.Publisher(
            '/checkpoint', NavSatFix, queue_size=1)

        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_setpoint_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top_callback(self, msg):
        self.obs_range_top = msg.ranges

    def range_finder_bottom_callback(self, msg):
        self.obs_range_bottom = msg.ranges

    def lat_to_x(self,input_latitude): return 110692.0702932625 * (input_latitude - 19)
    def long_to_y(self,input_longitude): return -105292.0089353767 * (input_longitude - 72)

    def x_to_lat_diff(self,input_x): return (input_x / 110692.0702932625)
    def y_to_long_diff(self,input_y): return (input_y / -105292.0089353767)

    def calculate_movement_in_plane(self, total_movement):
        '''This Function will take the drone in straight line towards destination'''

        specific_movement = [0, 0]      # movement in specific direction that is x and y

        # Applying symmetric triangle method
        specific_movement[0] = (total_movement * self.diff_xy[0]) / self.distance_xy
        specific_movement[1] = (total_movement * self.diff_xy[1]) / self.distance_xy
        
        return specific_movement

    def obstacle_avoid(self):
        '''For Processing the obtained sensor data and publishing required 
        checkpoint for avoiding obstacles'''
       
        data = self.obs_range_top
        self.movement_in_plane = [0, 0]

        # destination in x and y form
        self.current_location_xy = [self.lat_to_x(self.destination[0]), 
                self.long_to_y(self.destination[1])]

        self.destination_xy = [self.lat_to_x(self.current_location[0]), 
                self.long_to_y(self.current_location[1])]

        self.diff_xy = [self.destination_xy[0] - self.current_location_xy[0],
                self.destination_xy[1] - self.current_location_xy[1]]

        self.distance_xy = math.hypot(self.diff_xy[0], self.diff_xy[1])
        

        for obs_distance in data:
            if 12 <= obs_distance:
                self.movement_in_1D = 10
            elif 6 <= obs_distance < 12:
                self.movement_in_1D = 4
            else:
                self.movement_in_1D = 2.5

        if self.movement_in_1D >= self.distance_xy:
            self.movement_in_1D = self.distance_xy

        for i in range(len(data)-1):
            if data[i] <= self.obs_closest_range:
                if i % 2 != 0:
                    self.movement_in_plane[0] = data[i] - self.obs_closest_range
                    self.movement_in_plane[1] = self.movement_in_1D
                else:
                    self.movement_in_plane[0] = self.movement_in_1D
                    self.movement_in_plane[1] = data[i] - self.obs_closest_range
            else :
                self.movement_in_plane = self.calculate_movement_in_plane(self.movement_in_1D)
        
        print(self.movement_in_plane,self.movement_in_1D)

        self.checkpoint.latitude = self.current_location[0] - self.x_to_lat_diff(self.movement_in_plane[0])
        self.checkpoint.longitude = self.current_location[1] - self.y_to_long_diff(self.movement_in_plane[1])
        self.checkpoint.altitude = 25

        self.pub_checkpoint.publish(self.checkpoint)


if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.obstacle_avoid()
        rate.sleep()
