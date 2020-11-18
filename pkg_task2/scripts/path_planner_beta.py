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

        self.sample_time = 0.01

        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)

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

    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def obstacle_avoid(self):
        '''For Processing the obtained sensor data'''

        self.checkpoint.latitude = self.destination[0]
        self.checkpoint.longitude = self.destination[1]
        self.checkpoint.altitude = self.destination[2]

        diff_x = abs(self.lat_to_x(self.current_location[0] - self.destination[0]))
        diff_y = abs(self.long_to_y(self.current_location[1] - self.destination[1]))

        for i in range(len(self.obs_range_top)-1):
            print(self.obs_range_top)
            if self.obs_range_top[i] <= 1.5:
                if i % 2 == 0:
                    self.checkpoint.latitude = self.current_location[0]
                else:
                    self.checkpoint.longitude = self.current_location[1]

        self.pub_checkpoint.publish(self.checkpoint)


if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.obstacle_avoid()
        rate.sleep()
