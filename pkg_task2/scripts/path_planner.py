#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix, LaserScan, Imu

class PathPlanner():

    def __init__(self):
        rospy.init_node('path_planner_beta')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]

        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()

        # 
        self.obs_range_top = []
        self.obs_range_bottom = []

        # self.yaw_error = 0
        self.diff_range = [0, 0, 0, 0]
        self.movement_in_1D = 0
        self.movement_in_plane = [0, 0]     # [x, y]
        self.obs_closest_range = 2          # closest distance of obstacle (in meters)

        self.sample_time = 0.01

        # Publisher
        self.pub_checkpoint = rospy.Publisher(
            '/checkpoint', NavSatFix, queue_size=1)

        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_setpoint_callback)
        # rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
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
    def long_to_y(self,input_y): return -105292.0089353767 * (input_longitude - 72)
    def x_to_lat(self,input_y): return (input_x / 110692.0702932625) + 19
    def y_to_long(self,input_y): return (input_y / -105292.0089353767) + 72

    def obstacle_avoid(self):
        '''For Processing the obtained sensor data'''

        data = self.obs_range_top
        self.movement_in_plane = [0, 0]
        # self.checkpoint.latitude = self.destination[0]
        # self.checkpoint.longitude = self.destination[1]
        # self.checkpoint.altitude = self.destination[2]


        for obs_distance in data:
            if 12 <= obs_distance <= 25:
                self.movement_in_1D = 10
            elif 6 <= obs_distance < 12:
                self.movement_in_1D = 4
            else:
                self.movement_in_1D = 2

        self.movement_in_plane = self.calculate_movement_in_plane(self.movement_in_1D)

        for i in range(len(self.obs_range_top)-1):
            if self.obs_range_top[i] <= self.obs_closest_range:
                if i % 2 == 0:
                    self.movement_in_plane[0] += data[i] - self.obs_closest_range
                    self.movement_in_plane[1] = self.movement_in_1D
                else:
                    self.movement_in_plane[0] = self.movement_in_1D
                    self.movement_in_plane[1] += data[i] - self.obs_closest_range


        self.pub_checkpoint.publish(self.checkpoint)


if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.obstacle_avoid()
        rate.sleep()
