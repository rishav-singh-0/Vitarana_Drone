#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from std_msgs.msg import String,Float32
import std_msgs.msg


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
        self.status="DELIVERY"#it will be either "delevery" or "returns"
        self.pick_drop_box=False
        self.msg_from_marker_find=False
        self.cnt=0
        self.attech_situation = False
        # self.destination_list=[[18.9999864489,71.9999430161,8.44099749139],
        #                        [(18.9999864489+4*0.000013552),(71.9999430161+0.000014245),8.44099749139],
        #                        [(18.9999864489+0.000013552),(71.9999430161+0.000014245),8.44099749139],
        #                        [(18.9999864489+0.000013552),71.9999430161,8.44099749139]]

        self.dst=[0,0,0]
        self.container=[0,0,0]
        self.kaam_aabhi_baki_hai=False
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
        self.obs_range_bottom = []

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

        self.sample_time = 0.08

        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)
        self.grip_flag=rospy.Publisher('/gripp_flag',String,queue_size=1)
        self.destination_data=rospy.Publisher('/destination_data' , NavSatFix,queue_size=1)

        self.next_flag=rospy.Publisher('/next_destination_flag',Float32,queue_size=1)
        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_setpoint_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        rospy.Subscriber('/marker_error', NavSatFix, self.marker_error_callback)
        

        rospy.Subscriber('/box_checkpoint',NavSatFix,self.csv_checkpoint)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def csv_checkpoint(self,msg):
        self.status=msg.header.frame_id
        self.container=[msg.latitude,msg.longitude,msg.altitude]
        if(self.dst!=self.container):
            self.dst=self.container
            print(self.dst)


    def marker_error_callback(self, msg):
        self.img_data = [msg.latitude, msg.longitude]
        # print("holA")
        # print(msg.header.frame_id)


    def gripper_check_callback(self, state):
        self.attech_situation = state.data


    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top_callback(self, msg):
        self.obs_range_top = msg.ranges

    def range_finder_bottom_callback(self, msg):
        if(msg.ranges[0]>0.410000):
            self.obs_range_bottom = msg.ranges
            # print(self.obs_range_bottom[0])

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
        #print(self.pick_drop_box)
        # print(self.pause_process)
        # print(self.destination)
        # print("yoo",self.current_location)
        if -0.000005217 <= (self.destination[0]-self.current_location[0]) <= 0.000005217:
           
            if -0.0000033487 <= (self.destination[1]-self.current_location[1])<= 0.0000033487:
                self.pick_drop_box=True
               
                if(self.pause_process):
                    self.msg_from_marker_find=True
                if ((-0.02<=(self.destination[2]-self.current_location[2]) <= 0.05) and self.pick):
                    if(self.attech_situation):
                        self.reach_flag=True
                        self.pause_process=False
                        self.next_flag.publish(1.0)
                elif((self.obs_range_bottom[0]<=0.4500) and (not self.pick)):
                    if(self.attech_situation):
                            self.reach_flag=True
                            self.pause_process=False
                            self.next_flag.publish(1.0)
                        # self.next_flag.publish(1.0)
                        #print(self.pick_drop_box)
                        #if(not self.pick_drop_box):
                        # print(self.cnt)
                        # if(self.cnt==3):
                        #     self.cnt=3
                        # else:
                        #     self.cnt+=1
                        
                        #self.pick_drop_box=False
                    



    def obstacle_avoid(self):
        '''For Processing the obtained sensor data and publishing required
        checkpoint for avoiding obstacles'''
        # self.destination=self.destination_list[self.cnt]
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


        


        if(math.hypot((self.destination[0]-self.current_location[0]),(self.destination[1]-self.current_location[1]))<=6 and self.pick):
            self.checkpoint.altitude=self.destination[2]+4
            self.checkpoint.longitude=self.destination[1]
            self.checkpoint.latitude=self.destination[0]
            
        else:
            self.checkpoint.altitude = 12
        # self.altitude_control()
        self.desti_data.latitude=self.destination[0]
        self.desti_data.longitude=self.destination[1]
        self.desti_data.altitude=self.destination[2]


        # Publishing
        # if(not self.pick_drop_box):
        self.pub_checkpoint.publish(self.checkpoint)
        
        self.destination_data.publish(self.desti_data)

    def marker_find(self):

        # if(not self.sudo_destination_reach):
        #     if(self.img_data!=[0,0] and (not self.pause_process)):
        #         self.destination=[self.current_location[0]+self.x_to_lat_diff(self.img_data[0]),self.current_location[1]+self.y_to_long_diff(self.img_data[1])]
        #         self.pause_process=True
        # elif(self.sudo_destination_reach):
        # print(self.pause_process)
        if(self.img_data==[0,0] and (not self.pause_process)):
            self.checkpoint.altitude=self.current_location[2]+1
            self.pub_checkpoint.publish(self.checkpoint)
        elif(self.img_data!=[0,0] and (not self.pause_process)):
            print("yoooooooo")
            self.destination[0]=self.current_location[0]+self.x_to_lat_diff(self.img_data[0])
            self.destination[1]=self.current_location[1]+self.y_to_long_diff(self.img_data[1])
            self.checkpoint.latitude=self.destination[0]
            self.checkpoint.longitude=self.destination[1]

            self.pause_process=True
            # self.pub_checkpoint.publish(self.checkpoint)

    def pick_n_drop(self):
        
        self.checkpoint.altitude=self.destination[2]-0.08
        # self.pub_checkpoint.publish(self.checkpoint)
        if(self.reach_flag):
            #print("yoo")
            if(self.pick and self.attech_situation):
                self.grip_flag.publish('True')
                # self.next_flag.publish(1.0)
                self.pick=False
                
            else:
                self.grip_flag.publish('False')
                # self.next_flag.publish(1.0)
                self.pick=True

            self.reach_flag=False#not self.reach_flag
            self.pick_drop_box=False
            # self.next_flag.publish(1.0)
        #print(self.pick)
            
    def function_call(self):
        print("pick",self.pick)
        # print("msg_marker",self.msg_from_marker_find)
        # print("pause_process",self.pause_process)
        
        if(self.dst==[0,0,0]):
            return
        # print(self.msg_from_marker_find)
        if(not self.pause_process):
            self.destination=self.dst
        if(self.status=="DELIVERY"):
            if(not self.pick_drop_box):
                # print("obstacle avoid")
                self.msg_from_marker_find=False
                self.pause_process=False
                self.obstacle_avoid()
                # self.threshould_box()
            elif(self.pick_drop_box):
                if(not self.pick and not self.msg_from_marker_find):
                    # print("marker_find")
                    self.marker_find()
                    # self.threshould_box()
                elif(self.pick or self.msg_from_marker_find):
                    # print("pick n drop")
                    self.pick_n_drop()
                    # self.threshould_box()
                    #print("niche jao")
        elif(self.status=="RETURN"):
            if(not self.pick_drop_box):
                self.obstacle_avoid()
                # self.threshould_box()
                print("obstacle_avoid")
                #self.threshould_box()
            elif(self.pick_drop_box):
                self.pick_n_drop()
                # self.threshould_box()
                print("pick_n_drop")
        self.threshould_box()
        print(self.checkpoint.altitude)
        self.pub_checkpoint.publish(self.checkpoint)

if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.function_call()
        rate.sleep()
