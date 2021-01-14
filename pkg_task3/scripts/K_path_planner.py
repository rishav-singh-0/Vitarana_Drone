#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64, Int8
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from vitarana_drone.msg import MarkerData
 import csv


class PathPlanner():

    def __init__(self):
        rospy.init_node('path_planner_beta')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = []
        with open('manifest.csv','r') as x:
            content = csv.reader(x)
            # print(content)
            self.iter=0
            for i in content:
                self.destination.append(i)
                self.destination[self.iter].pop(0)
                self.iter+=1

                

        self.take_destination = False                   # Destination checking for obstacle avoid
        self.cnt = 0                                    # Formal counter for getting drone coordinates

        self.drone_co_ordinates = [0, 0, 0]             # Inital coordinates
        self.img_data = [0, 0]                          # For getting the x & y valuses from /marker_error
        self.pose_cnt = 0                               # For switching in markr_find condition
        self.check_marker = True                        # For switching in markr_find coordinates
        
        self.building_id = 0                            # Marking building IDs
        
        self.altitude_margin_up = 0.6                   # Margin for upper coordinates
        self.altitude_margin_down = 0.8                 # Margin for lower coordinates
        self.altitude_up = 3                            # Altitude addition upper coordinates
        self.altitude_down = 1                          # Altitude addition lower coordinates
        self.altitude_marker_find = 16.5                # Altitude for detecting marker
        self.driving_altitude = 5                       # Altitude for driving the drone
        
        # Converting latitude and longitude in meters for calculation
        self.destination_xy = [0, 0]

        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.current_location_xy = [0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()

        # Initializing to store data from Lazer Sensors
        self.obs_range_top = []
        self.obs_range_bottom = [0]
        self.obs_range_bottom[0]=0

        # Defining variables which are needed for calculation
        # diffrence of current and final position
        self.diff_xy = [0, 0]
        self.distance_xy = 0                # distance between current and final position

        self.movement_in_1D = 0             # maximum movement to be done in one direction
        # [x, y] -> movement distribution in x and y
        self.movement_in_plane = [0, 0]
        # closest distance of obstacle (in meters)
        self.obs_closest_range = 10

        # container for publishing marker_data
        self.cmd_marker_data = MarkerData()

        self.sample_time = 1

        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)
        self.pub_error_X=rospy.Publisher('/edrone/err_x_m', Float64, queue_size=1)
        self.pub_error_Y=rospy.Publisher('/edrone/err_y_m', Float64, queue_size=1)
        self.pub_building_no=rospy.Publisher('/edrone/curr_marker_id', Int8, queue_size=1)
        self.pub_marker_data = rospy.Publisher('/edrone/marker_data', MarkerData, queue_size=1)

        # Subscriber
        # rospy.Subscriber('/final_setpoint', NavSatFix, self.final_setpoint_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)
        rospy.Subscriber('/marker_error', NavSatFix, self.marker_error_callback)
    # def final_setpoint_callback(self, msg):
    #     self.destination = [msg.latitude, msg.longitude, msg.altitude]


    def marker_error_callback(self, msg):
        self.img_data = [msg.latitude, msg.longitude]

    def gps_callback(self, msg):
        if(msg.latitude != 0 and msg.longitude != 0):
            if(self.cnt == 0):
                self.drone_co_ordinates = [msg.latitude, msg.longitude, msg.altitude]
                self.current_location = [msg.latitude, msg.longitude, msg.altitude]
                self.cnt += 1
            self.current_location = [msg.latitude, msg.longitude, msg.altitude]
        

    def range_finder_top_callback(self, msg):
        self.obs_range_top = msg.ranges

    def range_finder_bottom_callback(self, msg):
         self.obs_range_bottom = msg.ranges

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
        specific_movement[0] = (
            total_movement * self.diff_xy[0]) / self.distance_xy
        specific_movement[1] = (
            total_movement * self.diff_xy[1]) / self.distance_xy
        return specific_movement

    def marker_error_publish(self):
        '''Publishing Marker Data'''
        self.pub_error_X.publish(self.img_data[0])
        self.pub_error_Y.publish(self.img_data[1])
        self.pub_building_no.publish(self.building_id + 1)

        self.cmd_marker_data.err_x_m = self.img_data[0]
        self.cmd_marker_data.err_y_m = self.img_data[1]
        self.cmd_marker_data.marker_id = self.building_id + 1
        self.pub_marker_data.publish(self.cmd_marker_data)

    def destination_check(self):
        ''' function will hendle all desired positions '''
        if -0.000010517 <= self.current_location[0]-self.destination[self.building_id][0] <= 0.000010517:
            if -0.0000127487 <= self.current_location[1]-self.destination[self.building_id][1] <= 0.0000127487:
                    self.take_destination = True

    def marker_box(self):
        if -0.000010517 <= self.current_location[0]-self.checkpoint.latitude <= 0.000010517:
            if -0.0000127487 <= self.current_location[1]-self.checkpoint.longitude <= 0.0000127487:
                if -0.2<=self.current_location[2]-self.checkpoint.altitude<=0.2 :
                    self.check_marker=True
                    if(self.pose_cnt==4):
                        self.building_id+=1
                        self.take_destination=not self.take_destination
                        self.pose_cnt=0


    def obstacle_avoid(self):
        '''For Processing the obtained sensor data and publishing required 
        checkpoint for avoiding obstacles'''

        if self.destination == [0, 0, 0]:
            return

        data = self.obs_range_top
        self.movement_in_plane = [0, 0]

        # destination in x and y form
        self.current_location_xy = [self.lat_to_x(self.destination[self.building_id][0]),
                                    self.long_to_y(self.destination[self.building_id][1])]

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
                self.movement_in_1D = 8 
            else:
                self.movement_in_1D = 2.5

        # checking if destination is nearer than maximum distance to be travelled
        if self.movement_in_1D >= self.distance_xy:
            self.movement_in_1D = self.distance_xy

        # doge the obstacle if its closer than certain distance
        for i in range(len(data)-1):
            if data[i] <= self.obs_closest_range:
                if i % 2 != 0:
                    self.movement_in_plane[0] = data[i] - self.obs_closest_range
                    self.movement_in_plane[1] = self.movement_in_1D
                else:
                    self.movement_in_plane[0] = self.movement_in_1D
                    self.movement_in_plane[1] = data[i] - self.obs_closest_range
            else:
                self.movement_in_plane = self.calculate_movement_in_plane(self.movement_in_1D)


        # setting the values to publish
        self.checkpoint.latitude = self.current_location[0] - self.x_to_lat_diff(self.movement_in_plane[0])
        self.checkpoint.longitude = self.current_location[1] - self.y_to_long_diff(self.movement_in_plane[1])
        # giving fixed altitude for now will work on it in future
        if(self.destination[self.building_id][2] > self.drone_co_ordinates[2]):
            if(self.obs_range_bottom[0] < self.altitude_up+self.altitude_margin_up):
                self.checkpoint.altitude = self.destination[self.building_id][2] + self.altitude_up
                self.checkpoint.latitude = self.current_location[0]
                self.checkpoint.longitude = self.current_location[1]
            else:
                self.checkpoint.altitude = self.destination[self.building_id][2] + self.altitude_up
            
        else:
            if(self.obs_range_bottom[0] < self.altitude_margin_down):
                self.checkpoint.altitude=self.current_location[2] + self.altitude_down
            elif(self.obs_range_bottom>2):
                self.checkpoint.altitude=self.current_location[2]
            else:
                self.checkpoint.altitude=self.destination[self.building_id][2] + self.altitude_down
        
        # Publishing
        self.pub_checkpoint.publish(self.checkpoint)

    def marker_find(self):
        '''algorithm for marker detecting'''

        if(self.pose_cnt==0):
            self.checkpoint.altitude = self.destination[self.building_id][2] + self.altitude_down
            self.pose_cnt += 1
            self.check_marker=not self.check_marker

        if(self.pose_cnt==1 and self.check_marker):
            self.checkpoint.altitude=self.destination[self.building_id][2] + self.altitude_marker_find
            self.checkpoint.latitude=self.destination[self.building_id][0]
            self.checkpoint.longitude=self.destination[self.building_id][1]
            self.pose_cnt+=1
            self.check_marker=not self.check_marker

        if(self.img_data[0]!=0.0 and self.pose_cnt==2 and self.check_marker):
            self.checkpoint.latitude=self.current_location[0]+self.x_to_lat_diff(self.img_data[0])
            self.checkpoint.longitude=self.current_location[1]+self.y_to_long_diff(self.img_data[1])
            self.pose_cnt+=1
            self.check_marker=not self.check_marker
        
        if(self.pose_cnt==3 and self.check_marker):
            self.checkpoint.altitude=self.destination[self.building_id][2] + self.driving_altitude
            self.pose_cnt+=1
            if(self.building_id==2):
                self.checkpoint.altitude=self.destination[self.building_id][2]
        
        self.pub_checkpoint.publish(self.checkpoint)
        return


if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.marker_error_publish()
        if(not planner.take_destination):
            planner.destination_check()
            planner.obstacle_avoid()
        else:
            planner.marker_find()
            planner.marker_box()

        rate.sleep()
