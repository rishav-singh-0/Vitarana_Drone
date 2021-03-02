#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from std_msgs.msg import String,Float32
import std_msgs.msg
import tf
from vitarana_drone.srv import *


class PathPlanner():

    def __init__(self):
        rospy.init_node('path_planner_beta')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.destination_xy = [0, 0]
        self.img_data = [0, 0]                               # Data which will come from the maeker_detect.py script
        self.pause_process = False                           # It will helpful to stop taking the data form the marker_detect
        self.reach_flag = False                              # For reaching at every position which is require threshould box
        self.pick = True                                     # For deciding wather to pick or drop a box
        self.status = "DELIVERY"                             # It will be either "delevery" or "returns"
        self.pick_drop_box = False                           # Switch for obstacle avoid and all funtion
        self.msg_from_marker_find = False                    # Flag for calling pick_n_drop() after marker_find()
        self.attech_situation = False                        # It will check if box is attechable
        self.dst = [0, 0, 0]                                 # It will take data from the scheduler algorithm
        self.container = [0, 0, 0]                           # Buffer for data coming from scheduler algorithm
        # For limiting the altitude due to current_location
        self.drone_orientation_quaternion = [0, 0, 0, 0]     # Drone orientation in qurtenion
        self.drone_orientation_euler = [0, 0, 0, 0]          # Drone orientation in euler
        ################################################################################
        self.buffer_altitude=0                               #for assigning altitude to the altitude variable
        self.altitude=0                                      #for assigning buffer_altitude
        self.limiter=0                                       #limiting assignment to the alttitude
        self.prev_altitude=16.757981                         #Differentianiting between current and previous altitude
        ################################################################################

        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.current_location_xy = [0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()
        self.desti_data = NavSatFix()

        # Initializing to store data from Lazer Sensors

        #It will collect data from top range sensor
        self.obs_range_top = [0,0,0,0]
        #It will collect data from bottom range sensor
        self.obs_range_bottom = []

        # Defining variables which are needed for calculation
        # Diffrence of current and final position
        self.diff_xy = [0, 0]
        self.distance_xy = 0                # Distance between current and final position

        self.movement_in_1D = 0             # Maximum movement to be done in one direction
        # [x, y] -> movement distribution in x and y
        self.movement_in_plane = [0, 0]
        # Closest distance of obstacle (in meters)
        self.obs_closest_range = 8

        self.direction_xy = [0, 0]
        self.sample_time = 0.5

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
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

        rospy.Subscriber('/box_checkpoint',NavSatFix,self.csv_checkpoint)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def gripper_client(self, check_condition):
        '''this function will call and wait for the gripper service'''

        rospy.wait_for_service('/edrone/activate_gripper')
        carry = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        msg_container = carry(check_condition)
        return msg_container.result     # true if box is atteched and visa versa

    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

    def csv_checkpoint(self,msg):
        self.status=msg.header.frame_id
        self.container=[msg.latitude,msg.longitude,msg.altitude]
        if(self.dst!=self.container):
            self.dst=self.container

    def marker_error_callback(self, msg):
        self.img_data = [msg.latitude, msg.longitude]

    def gripper_check_callback(self, state):
        self.attech_situation = state.data

    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top_callback(self, msg):
        if(-2.5<=(self.drone_orientation_euler[0]*180/3.14)<=2.5 and -2.5<=(self.drone_orientation_euler[1]*180/3.14)<=2.5):
            if(msg.ranges[0]>0.4 and msg.ranges[1]>0.4 and msg.ranges[2]>0.4 and msg.ranges[3]>0.4):
                self.obs_range_top = msg.ranges

    def range_finder_bottom_callback(self, msg):
        if(msg.ranges[0]>0.410000 or (self.destination[2]-self.current_location[2])<0.1):
            self.obs_range_bottom = msg.ranges

    #mehods for distance measurement
    def lat_to_x_diff(self,ip_lat_diff):return (110692.0702932625*ip_lat_diff)
    def long_to_y_diff(self,ip_long_diff):return (-105292.0089353767*ip_long_diff)

    # Functions for data conversion between GPS and meter with respect to origin
    def lat_to_x(self, input_latitude): return 110692.0702932625 * (input_latitude - 19)
    def long_to_y(self, input_longitude): return - 105292.0089353767 * (input_longitude - 72)

    def x_to_lat_diff(self, input_x): return (input_x / 110692.0702932625)
    def y_to_long_diff(self, input_y): return (input_y / -105292.0089353767)

    def threshould_box(self, limit):

        if -0.2 <= self.lat_to_x_diff(self.current_location[0]-self.destination[0])<= 0.2:

            if -0.2<= self.long_to_y_diff(self.current_location[1]-self.destination[1])<= 0.2:
                self.pick_drop_box=True

                if(self.pause_process):
                    self.msg_from_marker_find=True
                if (((-0.02<=(self.destination[2]-self.current_location[2]) <= 0.05) or (len(self.obs_range_bottom) and (self.obs_range_bottom[0]<=0.3940))) and self.pick ):
                    if(self.attech_situation):
                        self.reach_flag=True
                        self.pause_process=False
                        self.grip_flag.publish('True')
                        while( self.gripper_client(True)==False):

                            self.gripper_client(True)
                        self.pick=False
                        self.pick_drop_box=False
                        self.next_flag.publish(1.0)
                        while(self.destination==self.dst):
                            continue
                        self.destination=self.dst
                        self.buffer_altitude=self.current_location[2]-self.destination[2]
                        self.altitude=self.buffer_altitude
                        self.limiter=0

                elif((-2<(self.destination[2]-self.current_location[2]) < 2)and(self.obs_range_bottom[0]<=0.5100) and (not self.pick)):
                    if(self.attech_situation):
                            self.reach_flag=True
                            self.pause_process=False
                            self.grip_flag.publish('False')
                            # self.next_flag.publish(1.0)
                            self.pick=True
                            self.pick_drop_box=False
                            self.next_flag.publish(1.0)
                            while(self.destination==self.dst):
                                continue
                            self.destination=self.dst
                            self.buffer_altitude=self.current_location[2]-self.destination[2]
                            self.altitude=self.buffer_altitude
                            self.limiter=0

    def calculate_movement_in_plane(self, total_movement):
        '''This Function will take the drone in straight line towards destination'''

        specific_movement = [0, 0]      # movement in specific direction that is x and y
        

        # Applying symmetric triangle method
        specific_movement[0] = (total_movement * self.diff_xy[0]) / self.distance_xy
        specific_movement[1] = (total_movement * self.diff_xy[1]) / self.distance_xy
        return specific_movement
    def altitude_select(self):
        # bottom_distance = 4
        # alt_to_reach = self.destination[2] + 4
        # alt_present = self.current_location[2]
        # if (self.destination[2] > )
        # if (self.obs_range_bottom[0] < bottom_distance):
        if(self.limiter==0):
            # print("altitude")
            if((-0.2<=self.current_location[2]-self.destination[2]<=0.2) and self.distance_xy<30):
                self.checkpoint.altitude=self.destination[2]+2
                print("box grid")
            elif((-0.2<=self.current_location[2]-self.destination[2]<=0.2) and self.distance_xy>30):
                self.checkpoint.altitude=self.destination[2]+8
                print("box destination")
            else:
                if(self.current_location[2]<self.destination[2]):
                    print("curr<desti")
                    self.checkpoint.altitude=self.destination[2]+8
                else:
                    self.checkpoint.altitude=self.destination[2]+self.altitude+8
                    print("curr>desti")
            print(self.checkpoint.altitude)
            self.limiter+=1
            
    def obstacle_avoid(self):
        '''For Processing the obtained sensor data and publishing required
        checkpoint for avoiding obstacles'''
        if self.destination == [0, 0, 0]:
            return

        data = self.obs_range_top
        self.movement_in_plane = [0, 0]
        avoid_obs_in_x=0
        avoid_obs_in_y=0
        selected_lat=selected_long=0

        # destination in x and y form
        self.current_location_xy = [self.lat_to_x(self.destination[0]),
                                    self.long_to_y(self.destination[1])]

        self.destination_xy = [self.lat_to_x(self.current_location[0]),
                               self.long_to_y(self.current_location[1])]

        self.diff_xy = [self.destination_xy[0] - self.current_location_xy[0],
                        self.destination_xy[1] - self.current_location_xy[1]]
        # print("x,",self.diff_xy[0])
        # print("y",self.diff_xy[1])
        self.distance_xy = math.hypot(self.diff_xy[0], self.diff_xy[1])

        # calculating maximum distance to be covered at once
        for i in [0, 1]:
            d = data[self.direction_xy[i]]
            if d > 22:
                d = 22
            self.movement_in_1D = d * 0.75

        # print(self.movement_in_plane)
        
        ###########################################################################################
        if(self.diff_xy[0]>0):
            selected_lat=3
        else:
            selected_lat=1
        if(self.diff_xy[1]>0):
            selected_long=2
        else:
            selected_long=0
        # print("lat",selected_lat)
        # print("long",selected_long)
        if(self.distance_xy>self.obs_range_top[selected_lat] or self.distance_xy>self.obs_range_top[selected_long]):
            if(self.obs_range_top[selected_lat]>=self.obs_range_top[selected_long] and self.obs_range_top[selected_long]<=12):
                # print("hello")
                if(self.diff_xy[0]>0):
                    avoid_obs_in_x=2
                else:
                    avoid_obs_in_x=-2
                self.movement_in_1D=0
            elif(self.obs_range_top[selected_lat]<=self.obs_range_top[selected_long] and self.obs_range_top[selected_lat]<=12):
                if(self.diff_xy[1]>0):
                    avoid_obs_in_y=2
                else:
                    avoid_obs_in_y=-2
                self.movement_in_1D=0
            else:
                avoid_obs_in_x=avoid_obs_in_y=0

        ###########################################################################################
        if(self.distance_xy<=8.0):
            self.movement_in_1D = self.distance_xy
        
        self.movement_in_plane = self.calculate_movement_in_plane(self.movement_in_1D)
        # setting the values to publish
        self.checkpoint.latitude = self.current_location[0] - self.x_to_lat_diff(self.movement_in_plane[0]) - self.x_to_lat_diff(avoid_obs_in_x)
        self.checkpoint.longitude = self.current_location[1] - self.y_to_long_diff(self.movement_in_plane[1]) - self.y_to_long_diff(avoid_obs_in_y)
        self.altitude_select()
        # self.checkpoint.altitude = 25
        if(self.status=="RETURN"):
            if(self.pick==False):
                self.checkpoint.altitude=self.destination[2]+8
            else:
                self.checkpoint.altitude=self.destination[2]+2
        self.desti_data.latitude=self.destination[0]
        self.desti_data.longitude=self.destination[1]
        self.desti_data.altitude=self.destination[2]

        # Publishing
        self.pub_checkpoint.publish(self.checkpoint)
        self.destination_data.publish(self.desti_data)

    def marker_find(self):
        '''it will take data from the marker_detect.py and will do process on it'''
        if(self.img_data == [0, 0] and (not self.pause_process)):
            self.checkpoint.altitude = self.current_location[2]+1
            self.pub_checkpoint.publish(self.checkpoint)
        elif(self.img_data!=[0,0] and (not self.pause_process)):
            self.destination[0] = self.current_location[0] + self.x_to_lat_diff(self.img_data[0])
            self.destination[1] = self.current_location[1] + self.y_to_long_diff(self.img_data[1])
            self.checkpoint.latitude = self.destination[0]
            self.checkpoint.longitude = self.destination[1]
            self.pause_process = True

    def pick_n_drop(self):
        '''used to decrease the altitude'''
        self.checkpoint.altitude = self.destination[2]-0.2

    def function_call(self):
        '''it will handle fulction calling in all time'''
        if(self.dst == [0, 0, 0]):
            return

        if(not self.pause_process):
            self.destination = self.dst
        if(self.status == "DELIVERY"):
            if(not self.pick_drop_box):
                self.msg_from_marker_find = False
                self.pause_process = False
                self.obstacle_avoid()
            elif(self.pick_drop_box):
                self.limiter=0
                if(not self.pick and not self.msg_from_marker_find):
                    self.marker_find()
                elif(self.pick or self.msg_from_marker_find):
                    self.pick_n_drop()
        elif(self.status == "RETURN"):
            if(not self.pick_drop_box):
                self.obstacle_avoid()
            elif(self.pick_drop_box):
                self.pick_n_drop()
                self.limiter=0
        if(self.pick):
            self.threshould_box(0.2)
        elif(not self.pick):
            if(self.pause_process):
                self.threshould_box(0.25)
            else:
                self.threshould_box(0.58)
        self.pub_checkpoint.publish(self.checkpoint)


if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.function_call()
        rate.sleep()
