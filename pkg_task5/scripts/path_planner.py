#!/usr/bin/env python

'''
This python file runs a ROS-node 'path_planner' which takes care of the path to be followed
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /checkpoint             /marker_data
                                /edrone/gps
                                /edrone/range_finder_top
                                /edrone/range_finder_bottom

'''

import rospy
import math
from sensor_msgs.msg import NavSatFix, LaserScan, Imu


class PathPlanner():

    def __init__(self):
        rospy.init_node('path_planner')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.destination_xy = [0, 0]

        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.current_location_xy = [0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()

        # Initializing delivery/return coordinates 
        self.box_pose = [0, 0, 0]

        # Initializing to store data from Lazer Sensors
        self.obs_range_top = []
        # self.obs_range_bottom = []

        # Defining variables which are needed for calculation
        # diffrence of current and final position
        self.diff_xy = [0, 0]
        self.distance_xy = 0                # distance between current and final position

        self.movement_in_1D = 0             # maximum movement to be done in one direction
        # [x, y] -> movement distribution in x and y
        self.movement_in_plane = [0, 0]
        # closest distance of obstacle (in meters)
        self.obs_closest_range = 8
        
        self.direction_xy = [0, 0]          # direction to go in

        # Flags
        self.box_picked = False             # check if box is picked
        self.lock = False                   # lock the particular direction
        self.status = "DELIVERY"            # it will be either "delevery" or "returns"

        self.sample_time = 0.5

        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)

        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/box_checkpoint', NavSatFix, self.box_checkpoint_callback)
        # rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top_callback(self, msg):
        self.obs_range_top = msg.ranges

    # def range_finder_bottom_callback(self, msg):
    #     self.obs_range_bottom = msg.ranges

    def box_checkpoint_callback(self, msg):
        self.box_pose = [msg.latitude, msg.longitude, msg.altitude]

    # Functions for data conversion between GPS and meter with respect to origin
    def lat_to_x(self, input_latitude): return 110692.0702932625 * (input_latitude - 19)
    def long_to_y(self, input_longitude): return - 105292.0089353767 * (input_longitude - 72)

    def x_to_lat_diff(self, input_x): return (input_x / 110692.0702932625)
    def y_to_long_diff(self, input_y): return (input_y / -105292.0089353767)

    def calculate_movement_in_plane(self, total_movement):
        '''This Function will take the drone in straight line towards destination'''

        # movement in specific direction that is x and y
        specific_movement = [0, 0]

        # Applying symmetric triangle method
        specific_movement[0] = (total_movement * self.diff_xy[0]) / self.distance_xy
        specific_movement[1] = (total_movement * self.diff_xy[1]) / self.distance_xy
        return specific_movement

    def altitude_control(self):
        dist_z = self.current_location[2] - self.destination[2] + 3
        slope = dist_z / (self.distance_xy - 3)
        self.checkpoint.altitude = self.current_location[2] + (slope * dist_z)
    
    def threshould_box(self, limit):
        


    def obstacle_avoid(self):
        '''For Processing the obtained sensor data and publishing required
        checkpoint for avoiding obstacles'''

        if self.destination == [0, 0, 0]:
            return

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

        self.direction_xy[0] = 1 if self.diff_xy[0] < 0 else 3
        self.direction_xy[1] = 0 if self.diff_xy[1] < 0 else 2
        # print(self.direction_xy, self.diff_xy, data)

        # Calculating Maximum Possible movement possible in a particular direction
        for i in [0, 1]:
            d = data[self.direction_xy[i]] if data[self.direction_xy[i]] < 24 else 24
            self.movement_in_1D = d * 0.65

        # if(self.distance_xy <= 8.0):
        #     self.movement_in_1D = self.distance_xy

        for i in [0, 1]:
            if data[self.direction_xy[i]] < self.diff_xy[i]:
                self.movement_in_plane[i] = data[self.direction_xy[i]] * 0.75
            if self.movement_in_plane[i] < 4:
                self.movement_in_plane[i] = self.diff_xy[i]

        # self.movement_in_plane = [min(movement[0],movement[2]),min(movement[1],movement[3])]

        # for i in range(2):
        #     if (-0.5 <= self.destination_xy[i] - self.current_location_xy[i] <= 0.5
        #             and not self.lock and self.movement_in_plane[(i+1)%2] < abs(self.diff_xy[(i+1)%2])):
        #         self.lock = True

            # if self.lock:
            #     # if direction is locked then take jump of 3 meters and check if way is clear
            #     self.destination[i] += 3

        # print(self.movement_in_plane, movement)

        # setting the values to publish
        self.checkpoint.latitude = self.current_location[0] - self.x_to_lat_diff(self.movement_in_plane[0])
        self.checkpoint.longitude = self.current_location[1] - self.y_to_long_diff(self.movement_in_plane[1])
        self.checkpoint.altitude = 24
        # self.altitude_control()

        # Publishing
        self.pub_checkpoint.publish(self.checkpoint)
        
    def function_call(self):
        self.obstacle_avoid()


if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.obstacle_avoid()
        planner.function_call()
        rate.sleep()
