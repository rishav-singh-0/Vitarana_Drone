#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from std_msgs.msg import String


class PathPlanner():

    def __init__(self):
        rospy.init_node('path_planner_beta')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.destination_xy = [0, 0]
        
        #*******************************************opt********************************#
        self.sudo_destination_reach=False#checking if its reached at the destination which is for delevery in csv file
        self.desired_destination=[0,0,0]#giving it to the threshould box if it has found marker
        #above 2 will being erased:::::)
        self.img_data=[0,0]#data which will come from the maeker_detect.py script
        self.pause_process=False#it will helpful to stop taking the data form the marker_detect and focus on destination reach
        self.reach_flag=False#for reaching at every position which is require threshould box
        self.pick=True#for deciding wather to pick or drop a box
        self.status="D"#it will be either "delevery" or "returns"
        self.pick_drop_box=False
        self.msg_from_marker_find=False
        self.cnt=0
        self.destination_list=[[18.9999864489,71.9999430161,8.44099749139],
                               [18.9993676146,71.9999999999,10.854960]]
        #*******************************************opt********************************#
        
        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.current_location_xy = [0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()
        self.desti_data=NavSatFix()

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
        self.lock = False

        self.sample_time = 0.01

        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)
        self.grip_flag=rospy.Publisher('/gripp_flag',String,queue_size=1)
        self.destination_data=rospy.Publisher('/destination_data' , NavSatFix,queue_size=1)

        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_setpoint_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        # rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top_callback(self, msg):
        self.obs_range_top = msg.ranges

    # def range_finder_bottom_callback(self, msg):
    #     self.obs_range_bottom = msg.ranges

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

    def threshould_box(self):

        if -0.000010217 <= (self.destination[0]-self.current_location[0]) <= 0.000010217:
            if -0.0000037487 <= (self.destination[0]-self.current_location[1])<= 0.0000037487:
                self.pick_drop_box=True
                if(self.pause_process):
                    self.msg_from_marker_find=True
                if (-0.1<= (self.destination[2]-self.current_location[2]) <= 0.1):
                    self.reach_flag=True
                    if(self.cnt<2):
                        self.cnt+=1
                    



    def obstacle_avoid(self):
        '''For Processing the obtained sensor data and publishing required
        checkpoint for avoiding obstacles'''
        self.destination=self.destination_list[self.cnt]
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

        # calculating maximum distance to be covered at once
        # it can be done more efficiently using another pid
        for obs_distance in data:
            if 16 <= obs_distance:
                self.movement_in_1D = 15
            elif 9 <= obs_distance:
                self.movement_in_1D = 4
            else:
                self.movement_in_1D = 2.5

        # checking if destination is nearer than maximum distance to be travelled
        if self.movement_in_1D >= self.distance_xy:
            self.movement_in_1D = self.distance_xy

        # doge the obstacle if its closer than certain distance
        for i in range(len(data)-1):
            if data[i] <= self.obs_closest_range:
                if i % 2 != 0:
                    self.movement_in_plane[0] = data[i] - \
                        self.obs_closest_range
                    self.movement_in_plane[1] = self.movement_in_1D
                else:
                    self.movement_in_plane[0] = self.movement_in_1D
                    self.movement_in_plane[1] = data[i] - \
                        self.obs_closest_range
            else:
                self.movement_in_plane = self.calculate_movement_in_plane(
                    self.movement_in_1D)

        # print(self.movement_in_plane,self.movement_in_1D)

        # setting the values to publish
        self.checkpoint.latitude = self.current_location[0] - \
            self.x_to_lat_diff(self.movement_in_plane[0])
        self.checkpoint.longitude = self.current_location[1] - self.y_to_long_diff(
            self.movement_in_plane[1])


        print("almost done")


        if(math.hypot((self.destination[0]-self.current_location[0]),(self.destination[1]-self.current_location[1]))<=6 and self.pick):
            self.checkpoint.altitude=self.destination[2]+2
        else:
            self.checkpoint.altitude = 24
        # self.altitude_control()
        self.desti_data.latitude=self.destination[0]
        self.desti_data.longitude=self.destination[1]
        self.desti_data.altitude=self.destination[2]


        # Publishing
        self.pub_checkpoint.publish(self.checkpoint)
        self.destination_data.publish(self.desti_data)

    def marker_find(self):

        # if(not self.sudo_destination_reach):
        #     if(self.img_data!=[0,0] and (not self.pause_process)):
        #         self.destination=[self.current_location[0]+self.x_to_lat_diff(self.img_data[0]),self.current_location[1]+self.y_to_long_diff(self.img_data[1])]
        #         self.pause_process=True
        # elif(self.sudo_destination_reach):
        if(self.img_data==[0,0] and (not self.pause_process)):
            self.checkpoint.altitude=self.current_location[2]+4
        elif(self.img_data!=[0,0] and (not self.pause_process)):
            self.destination=[self.current_location[0]+self.x_to_lat_diff(self.img_data[0]),self.current_location[1]+self.y_to_long_diff(self.img_data[1])]
            self.pause_process=True

    def pick_n_drop(self):
        self.checkpoint.altitude=self.destination[2]
        self.pub_checkpoint.publish(self.checkpoint)
        if(self.reach_flag):
            if(self.pick):
                self.grip_flag.publish('True')
                self.pick=False
            else:
                self.grip_flag.publish('False')
                self.pick=True
            self.reach_flag=False#not self.reach_flag
            self.pick_drop_box=False
            

        pass

if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        if(planner.status=="D"):
            if(not planner.pick_drop_box):
                planner.obstacle_avoid()
            elif(planner.pick_drop_box):
                if(planner.pick or planner.msg_from_marker_find):
                    planner.pick_n_drop()
                elif(not planner.pick or not planner.msg_from_marker_find):
                    planner.marker_find()
        elif(planner.status=="R"):
            if(planner.pick_drop_box):
                planner.obstacle_avoid()
            elif(not planner.pick_drop_box):
                planner.pick_n_drop()

        planner.obstacle_avoid()
        rate.sleep()
