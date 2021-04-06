
#!/usr/bin/env python

'''
This python file runs a ROS-node of name path_planner which controls the path to be travelled in order to 
reach required destination which is given by scheduler script 
This node publishes and subsribes the following topics:
        PUBLICATIONS                SUBSCRIPTIONS                
        /gripp_flag                 /edrone/gps
        /destination_data           /edrone/range_finder_top
        /next_destination_flag      /edrone/range_finder_bottom
        /checkpoint                 /edrone/gripper_check
                                    /marker_error
                                    /edrone/imu_data
                                    /box_checkpoint

'''

#Team ID:            0983
#Theme:              VITARANA DRONE
#Author List:        Rishav Singh, Kashyap Joshi
#Filename:           path_planner.py
#Functions:          destination_callback,gps_callback,range_finder_bottom_callback,image_callback,detect_marker
#Global Variables:   None

import rospy
import math
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from std_msgs.msg import String, Float32
import std_msgs.msg
import tf
from vitarana_drone.srv import *


class PathPlanner():
    '''
    Purpose:
    ---
    This class will contain member functions to control below functionings.
    -->provide propper destination to eDrone.
    -->attech and detech boxes.
    -->decreace altitude for attwch and detech box.
    -->avoid obstacles.
    -->getting data from marker_detect.

    Input Argument:
    ---
    None
    '''

    def __init__(self):
        rospy.init_node('path_planner')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.destination_xy = [0, 0]
        # Data which will come from the maeker_detect.py script
        self.img_data = [0, 0]
        # It will helpful to stop taking the data form the marker_detect
        self.pause_process_after_detection = False
        # For deciding wather to pick or drop a box
        self.pick = True
        # It will be either "delevery" or "returns"
        self.status = "DELIVERY"
        # Switch for obstacle avoid and all funtion
        self.pick_drop_box = False
        # Flag for calling pick_n_drop() after marker_find()
        self.msg_from_marker_find = False
        # It will check if box is attechable
        self.attech_situation = False
        # It will take data from the scheduler algorithm
        self.incoming_distance = [0, 0, 0]
        # Buffer for data coming from scheduler algorithm
        self.container = [0, 0, 0]
        # For limiting the altitude due to current_location
        self.drone_orientation_quaternion = [0, 0, 0, 0]     # Drone orientation in qurtenion

        self.drone_orientation_euler = [0, 0, 0, 0]          # Drone orientation in euler
        self.buffer_altitude = 0                             # For assigning altitude to the altitude variable
        self.altitude = 0                                    # For assigning buffer_altitude
        self.limiter = 0                                     # Limiting assignment to the alttitude
        self.threshould_altitude=8.5                         #setting altitude threshould to 8.5

        # Present Location of the Drone Note
        self.current_location = [0, 0, 0]
        # Converting latitude and longitude in meters for calculation
        self.current_location_xy = [0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()
        self.desti_data = NavSatFix()

        # Initializing to store data from Lazer Sensors
        # ---
        # It will collect data from top range sensor
        self.obs_range_top = [0, 0, 0, 0]
        # It will collect data from bottom range sensor
        self.obs_range_bottom = []


        # Defining variables which are needed for calculation
        # ---
        # Diffrence of current and final position
        # It will useful to calculate x and y difference of destination from eDrone
        self.diff_xy = [0, 0]               # Difference between current location and destonation in lattitude and longitude form
        self.distance_xy = 0                # Distance between current and final position

        self.movement_in_1D = 0             # Maximum movement to be done in one direction

        # [x, y] -> movement distribution in x and y
        self.movement_in_plane = [0, 0]
        # Closest distance of obstacle (in meters)
        self.obs_closest_range = 8

        self.direction_xy = [0, 0]          # It will useful for storing the required data from the 'range finder top'
        self.sample_time = 0.5              # Defining the sample time

        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)
        self.grip_flag = rospy.Publisher('/gripp_flag', String, queue_size=1)
        self.destination_data = rospy.Publisher('/destination_data', NavSatFix, queue_size=1)
        self.next_flag = rospy.Publisher('/next_destination_flag', Float32, queue_size=1)

        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        rospy.Subscriber('/marker_error', NavSatFix, self.marker_error_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/box_checkpoint', NavSatFix, self.csv_checkpoint)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def gripper_client(self, check_condition):
        '''
        Purpose:
        ---
        Basically this function is made for service call and it will wait for service to active.
        when service will active it will send request to grip or drop box and receive appropreat feedback from the service.
        
        Input Argument:
        ---
        check_condition: [boolean]
            Input for gripping and ungrippig the parcel box

        Returns:
        ---
        msg_container.result(its reasult wather box is gripped or not as a feedback)

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/edrone/activate_gripper' service .
        '''

        rospy.wait_for_service('/edrone/activate_gripper')
        carry = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        msg_container = carry(check_condition)
        return msg_container.result     # true if box is atteched and visa versa

    def imu_callback(self, msg):
        '''
        Purpose:
        ---
        This is callback function function for saving the IMU data so that we can introduce limitation in taking the samples 
        from the 'range_finder_top'.
        
        When in latitude and longitude angle in euler is <= 2.5 apperars variables will start to sample the data 
        so unwanted noise will being removed due to to the drone inclination.

        Input Argument:
        ---
        msg : [ Imu ]
            Raw IMU data from IMU sensor

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/edrone/imu/data' topic.
        '''

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

    def csv_checkpoint(self, msg):
        '''
        Purpose:
        ---
        It will receive the coordinates of next destination which will being published by the shaduler algorithm.
        
        Input Argument:
        ---
        msg : [ NavSatFix ]
            GPS coordinate and box status

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/box_checkpoint' topic.
        '''

        self.status = msg.header.frame_id
        self.container = [msg.latitude, msg.longitude, msg.altitude]
        if(self.incoming_distance != self.container):
            self.incoming_distance = self.container

    def marker_error_callback(self, msg):
        '''
        Purpose:
        ---
        It will receive the difference of x and y coordinates from eDrone to cross marker from marker_detect script.
        
        Input Argument:
        ---
        msg : [ NavSatFix ]
            Diffrence of current and detected marker distance

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/marker_error' topic.
        '''

        self.img_data = [msg.latitude, msg.longitude]

    def gripper_check_callback(self, state):
        '''
        Purpose:
        ---
        It will receive the flag wather the box is attechable or not.
        True      -->    if attechable
        False     -->    if not attachable

        Input Argument:
        ---
        msg : [ String ]
            State of availabelity to grip the box

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/edrone/gripper_check' topic.
        '''
        self.attech_situation = state.data

    def gps_callback(self, msg):
        '''
        Purpose:
        ---
        Stores the data given by the gps sensor inside gps_position

        Input Arguments:
        ---
        msg :  [ NavSatFix ]
            raw data from gps sensor

        Returns:
        ---
        None

        Example call:
        ---
        gps_callback(<gps_data>)
        '''

        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top_callback(self, msg):
        '''
        Purpose:
        ---
        It will receive the data from range finder top sensor which is distance measurin sensor which is having max range if 25 meters for all 4 sides.
        In this function we filtered the data.
        When in latitude and longitude angle(basically in roll and pitch of eDrone.) in euler is <= 2.5 apperars variables will start to sample the data so unwanted noise will being removed due to to the drone inclination.
        Some unwanted noise on the threshould of <=0.4 is also removed
        
        Input Argument:
        ---
        msg : [ LaserScan ]
            List of distance from the obstacle if detected in either 4 directions

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/edrone/range_finder_top' topic.
        '''

        if(-2.5 <= (self.drone_orientation_euler[0]*180/3.14) <= 2.5 and -2.5 <= (self.drone_orientation_euler[1]*180/3.14) <= 2.5):
            if(msg.ranges[0] > 0.4 and msg.ranges[1] > 0.4 and msg.ranges[2] > 0.4 and msg.ranges[3] > 0.4):
                self.obs_range_top = msg.ranges

    def range_finder_bottom_callback(self, msg):
        '''
        Purpose:
        ---
        It will receive the data from the range finder bottom sensor.

        Input Argument:
        ---
        msg : [ LaserScan ]
            List of single element containing distance of eDrone and bottom hinderance

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the '/edrone/range_finder_bottom' topic.
        '''

        #Noise is being removed from this arrangement and this data is used for gripping and ungripping the parcel box.
        if(msg.ranges[0] > 0.410000 or (self.destination[2]-self.current_location[2]) < 0.1):
            self.obs_range_bottom = msg.ranges

    # mehods for distance measurement(conversion of difference in gps coordinates to cartesian form)
    #---
    def lat_to_x_diff(self, ip_lat_diff): 
        '''
        Purpose:
        ---
        Convert diffrence of GPS Latitude to x in meters 
        It will accepts one argument which is difference in latitude

        Input Argument:
        ---
        ip_lat_diff : [ float32 ]
            input GPS Latitude Coordinate 

        Returns:
        ---
        x value in meters

        Example call:
        ---
        lat_to_x_diff(0.0000134)
        '''
        return (110692.0702932625*ip_lat_diff)

    def long_to_y_diff(self, ip_long_diff): 
        '''
        Purpose:
        ---
        Convert diffrence of GPS Longitude to y in meters 
        It will accepts one argument which is difference in Longitude

        Input Argument:
        ---
        ip_long_diff : [ float32 ]
            input GPS Longitude Coordinate 

        Returns:
        ---
        y value in meters

        Example call:
        ---
        long_to_y_diff(0.0000134)
        '''
        return (-105292.0089353767*ip_long_diff)

    # Functions for data conversion between GPS and meter with respect to origin
    #---
    def lat_to_x(self, input_latitude): 
        '''
        Purpose:
        ---
        It will convert Latitude in x coordinates in meters

        Input Argument:
        ---
        input_latitude : [ float32 ]
            input GPS Latitude Coordinate 

        Returns:
        ---
        x value in meters

        Example call:
        ---
        lat_to_x_diff(19.0000134)
        '''
        return 110692.0702932625 * (input_latitude - 19)
    
    def long_to_y(self, input_longitude): 
        '''
        Purpose:
        ---
        Convert diffrence of GPS Latitude to x in meters 
        It will accepts one argument which is difference in latitude

        Input Argument:
        ---
        input_longitude : [ float32 ]
            input GPS Longitude Coordinate 

        Returns:
        ---
        y value in meters

        Example call:
        ---
        long_to_y(72.0000134)
        '''

        return - 105292.0089353767 * (input_longitude - 72)

    # Functions which will convert cartesian difference in gps accepted format
    #---
    def x_to_lat_diff(self, input_x): 
        '''
        Purpose:
        ---
        Convert meters (x) in GPS Latitude 

        Input Argument:
        ---
        input_x : [ float32 ]
            input x in meters

        Returns:
        ---
        GPS Latitude (relative to 19.0)

        Example call:
        ---
        lat_to_x_diff(10)
        '''
        return (input_x / 110692.0702932625)

    def y_to_long_diff(self, input_y): 
        '''
        Purpose:
        ---
        Convert meters (y) in GPS Longitude 

        Input Argument:
        ---
        input_y : [ float32 ]
            input y in meters

        Returns:
        ---
        GPS Longitude (relative to 72.0)

        Example call:
        ---
        lat_to_x_diff(25)
        '''
        return (input_y / -105292.0089353767)

    def threshould_box(self, limit):
        '''
        Purpose:
        ---
        It will make sure that deone will arive in propper threshould to minimize the proportion of error.
        Its applied to latitude,longitude and altitude.
        It will change some flags to invoke appropreat function at a perticuler event.
        When eDrone will come in threshould of latitude and longitude eDrone will start to decrease its altitude or start to find marker with respective function.
        When eDrone will arrive in threshould of altitude, It will drop or lift the box and publish the flag for new destination.

        Input Argument:
        ---
        limit : [ float32 ]
            cheching value of minima and maxima

        Returns:
        ---
        None

        Example call:
        ---
        threshould_box(0.25)
        '''

        if -0.2 <= self.lat_to_x_diff(self.current_location[0]-self.destination[0]) <= 0.2:

            if -0.2 <= self.long_to_y_diff(self.current_location[1]-self.destination[1]) <= 0.2:
                # True  --> marker_find() or pick_n_drop() will being called
                # False --> obstacle_avoid will being called
                self.pick_drop_box = True
                
                # 'self.pause_process_after_detection' is bool type and becomes true when marker will being detected
                if(self.pause_process_after_detection):
                    # 'self.msg_from_marker_find' will help to know that marker is detected or not
                    self.msg_from_marker_find = True

                if (((-0.02 <= (self.destination[2]-self.current_location[2]) <= 0.05) or (len(self.obs_range_bottom) and (self.obs_range_bottom[0] <= 0.3940))) and self.pick):
                    if(self.attech_situation):
                        self.pause_process_after_detection = False
                        self.grip_flag.publish('True')
                        while(self.gripper_client(True) == False):
                            self.gripper_client(True)
                        self.pick = False
                        self.pick_drop_box = False
                        self.next_flag.publish(1.0)
                        while(self.destination == self.incoming_distance):
                            continue
                        self.destination = self.incoming_distance
                        self.buffer_altitude = self.current_location[2] - self.destination[2]
                        self.altitude = self.buffer_altitude
                        self.limiter = 0

                elif((-2 < (self.destination[2]-self.current_location[2]) < 2) and (self.obs_range_bottom[0] <= 0.5100) and (not self.pick)):
                    if(self.attech_situation):
                        self.pause_process_after_detection = False
                        self.grip_flag.publish('False')
                        self.pick = True
                        self.pick_drop_box = False
                        # Punbishing for getting next destination
                        self.next_flag.publish(1.0)

                        # While box will not being atteched it will continue and eDrone won't move
                        while(self.destination == self.incoming_distance):
                            continue
                        self.destination = self.incoming_distance
                        # Below will store difference between current and desired location
                        self.buffer_altitude = self.current_location[2] - self.destination[2]
                        self.altitude = self.buffer_altitude
                        # while 'self.limiter'== 0 its permited to set altitude
                        self.limiter = 0

    def calculate_movement_in_plane(self, total_movement):
        '''
        Purpose:
        ---
        This Function will take the drone in straight line towards destination by bot latitude side and longitude side.

        Input Argument:
        ---
        total_movement : [ float32 ]
            Distance in meters

        Returns:
        ---
        specific_movement : [ float32 ]
            Distance in meters

        Example call:
        ---
        calculate_movement_in_plane(25)
        '''
        # movement in specific direction that is x and y
        specific_movement = [0, 0]

        # Applying symmetric triangle method
        specific_movement[0] = (total_movement * self.diff_xy[0]) / self.distance_xy
        specific_movement[1] = (total_movement * self.diff_xy[1]) / self.distance_xy
        return specific_movement

    def altitude_select(self):
        '''
        Purpose:
        ---
        This Function will help to select appropreat altitude for eDrone.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        altitude_select()
        '''

        if(self.limiter == 0):
            if((-0.2 <= self.current_location[2]-self.destination[2] <= 0.2) and self.distance_xy < 30):
                self.checkpoint.altitude = self.destination[2]+2

            elif((-0.2 <= self.current_location[2]-self.destination[2] <= 0.2) and self.distance_xy > 30):
                self.checkpoint.altitude = self.destination[2]+self.threshould_altitude

            else:
                if(self.current_location[2] < self.destination[2]):

                    self.checkpoint.altitude = self.destination[2]+self.threshould_altitude
                else:
                    self.checkpoint.altitude = self.destination[2] + self.altitude+self.threshould_altitude

            self.limiter += 1

    def obstacle_avoid(self):
        '''
        Purpose:
        ---
        For Processing the obtained sensor data and publishing required checkpoint for avoiding obstacles.
        It will also take care of altitude if needed.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        obstacle_avoid()
        '''

        if self.destination == [0, 0, 0]:
            return

        data = self.obs_range_top
        self.movement_in_plane = [0, 0]

        # Explaination of 'avoid_obs_in_xy' is given below
        # ---
        # avoid_obs_in_xy--> obstacle in x direction
        # avoid_obs_in_xy--> obstacle in y direction
        avoid_obs_in_xy = [0, 0]

        # Explaination of 'selected_latitude_and_longituide' is given below
        # ---
        # selected_latitude_and_longituide[0]--> move in x direction by specific value
        # selected_latitude_and_longituide[1]--> move in y direction by specific value
        selected_latitude_and_longituide = [0, 0]

        # Destination in x and y form(specificly in meters)
        self.current_location_xy = [self.lat_to_x(self.destination[0]),
                                    self.long_to_y(self.destination[1])]

        self.destination_xy = [self.lat_to_x(self.current_location[0]),
                               self.long_to_y(self.current_location[1])]

        # Calculating difference in form of x and y
        self.diff_xy = [self.destination_xy[0] - self.current_location_xy[0],
                        self.destination_xy[1] - self.current_location_xy[1]]


        # It will calculate straight line distance between current location and destination
        self.distance_xy = math.hypot(self.diff_xy[0], self.diff_xy[1])

        # calculating maximum distance to be covered at once
        for i in [0, 1]:
            required_distance = data[self.direction_xy[i]]
            if required_distance > 22:
                required_distance = 22
            if self.distance_xy > 22:
                self.movement_in_1D = required_distance
            else:
                self.movement_in_1D = required_distance * 0.65

        # Selecting appropreat index for list comming from range finder top to avoid obstacle
        if(self.diff_xy[0] > 0):
            selected_latitude_and_longituide[0] = 3
        else:
            selected_latitude_and_longituide[0] = 1
        if(self.diff_xy[1] > 0):
            selected_latitude_and_longituide[1] = 2
        else:
            selected_latitude_and_longituide[1] = 0

        # Deciding condition when to avoid obstacle(basically threshould defination in condition)
        if(self.distance_xy > self.obs_range_top[selected_latitude_and_longituide[0]] or self.distance_xy > self.obs_range_top[selected_latitude_and_longituide[1]]):
            if(self.obs_range_top[selected_latitude_and_longituide[0]] >= self.obs_range_top[selected_latitude_and_longituide[1]] and self.obs_range_top[selected_latitude_and_longituide[1]] <= 15):
                
                #Selecting appropreat distance to travel while avoiding obstacle
                if(self.diff_xy[0] > 0):
                    avoid_obs_in_xy[0] = 4
                else:
                    avoid_obs_in_xy[0] = -4
                self.movement_in_1D = 10

            # Deciding condition when to avoid obstacle(basically threshould defination in condition)
            elif(self.obs_range_top[selected_latitude_and_longituide[0]] <= self.obs_range_top[selected_latitude_and_longituide[1]] and self.obs_range_top[selected_latitude_and_longituide[0]] <= 15):
                
                # Deciding condition when to avoid obstacle(basically threshould defination in condition)
                if(self.diff_xy[1] > 0):
                    avoid_obs_in_xy[1] = 4
                else:
                    avoid_obs_in_xy[1] = -4
                self.movement_in_1D = 0
            else:
                avoid_obs_in_xy[0] = avoid_obs_in_xy[1] = 0

        if(self.distance_xy <= 8.0):
            self.movement_in_1D = self.distance_xy

        self.movement_in_plane = self.calculate_movement_in_plane(
            self.movement_in_1D)

        # Setting the values to publish
        self.checkpoint.latitude = self.current_location[0] - self.x_to_lat_diff(self.movement_in_plane[0]) - self.x_to_lat_diff(avoid_obs_in_xy[0])
        self.checkpoint.longitude = self.current_location[1] - self.y_to_long_diff(self.movement_in_plane[1]) - self.y_to_long_diff(avoid_obs_in_xy[1])
        self.altitude_select()
        # Checkin and correcting the provided altitude
        if(self.status == "RETURN"):
            if(self.pick == False):
                self.checkpoint.altitude = self.destination[2]+8
            else:
                self.checkpoint.altitude = self.destination[2]+8.5
       
        self.desti_data.latitude=self.destination[0]
        self.desti_data.longitude=self.destination[1]
        self.desti_data.altitude=self.destination[2]
        # Publishing the checkpoint at where eDrone have to reach
        self.pub_checkpoint.publish(self.checkpoint)
        self.destination_data.publish(self.desti_data)

    def marker_find(self):
        '''
        Purpose:
        ---
        It will take data from the marker_detect.py and will do process on it.
        It will take difference from cross marker to eDrone in x and y form.
        It will increase altitude by 1 meters if difference is not being published

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        marker_find()
        '''

        if(self.img_data == [0, 0] and (not self.pause_process_after_detection)):
            self.checkpoint.altitude = self.current_location[2]+1
            self.pub_checkpoint.publish(self.checkpoint)
        elif(self.img_data != [0, 0] and (not self.pause_process_after_detection)):
            self.destination[0] = self.current_location[0] + self.x_to_lat_diff(self.img_data[0])
            self.destination[1] = self.current_location[1] + self.y_to_long_diff(self.img_data[1])
            self.checkpoint.latitude = self.destination[0]
            self.checkpoint.longitude = self.destination[1]
            self.pause_process_after_detection = True

    def pick_n_drop(self):
        '''
        Purpose:
        ---
        Used to decrease the altitude for picking and dropping the box.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        Called in function_call() if needed.

        pick_n_drop()
        '''

        self.checkpoint.altitude = self.destination[2]-0.2

    def function_call(self):
        '''
        Purpose:
        ---
        This function will call perticular function on perticular event.
        It will publish checkpoints too.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        function_call()
        '''

        if(self.incoming_distance == [0, 0, 0]):
            return

        if(not self.pause_process_after_detection):
            self.destination = self.incoming_distance
        if(self.status == "DELIVERY"):
            if(not self.pick_drop_box):
                self.msg_from_marker_find = False
                self.pause_process_after_detection = False
                self.obstacle_avoid()
            elif(self.pick_drop_box):
                self.limiter = 0
                if(not self.pick and not self.msg_from_marker_find):
                    self.marker_find()
                elif(self.pick or self.msg_from_marker_find):
                    self.pick_n_drop()
        elif(self.status == "RETURN"):
            if(not self.pick_drop_box):
                self.obstacle_avoid()
            elif(self.pick_drop_box):
                self.pick_n_drop()
                self.limiter = 0
        if(self.pick):
            self.threshould_box(0.2)
        elif(not self.pick):
            if(self.pause_process_after_detection):
                self.threshould_box(0.25)
            else:
                self.threshould_box(1.5)
        self.pub_checkpoint.publish(self.checkpoint)

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the Edrone class's function function_call() for managing all the 
#                   function call to make possible drone to avoid obstacle,find marker,drop and lift the box.
if __name__ == "__main__":
    planner = PathPlanner()
    rate = rospy.Rate(1/planner.sample_time)
    while not rospy.is_shutdown():
        planner.function_call()
        rate.sleep()