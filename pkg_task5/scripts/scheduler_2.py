#!/usr/bin/env python
'''
#Team ID:            0983
#Theme:              VITARANA DRONE
#Author List:        Rishav Singh,Kashyap Joshi
#Filename:           scheduler_2.py
#Functions:          next_destination_callback,lat_to_x_diff,long_to_y_diff,read_and_set_data,data_publish
#Global Variables:   None
'''

'''
This python file runs a ROS-node 'data_processing' which takes care of the next destination to be reached
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /box_checkpoint         /next_destination_flag


'''
#print("testing phase")
import rospy
import math
import csv
import numpy
import os
import std_msgs.msg
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix


class Data_processing():

    '''
    Purpose:
    ---
    This class will contain member functions to process the .csv fileand publish it.

    Input Argument:
    ---
    None
    '''

    def __init__(self):
        rospy.init_node('data_processing')
        #data for eDrone coordinates is directily taken from provided grid data not hardcoded
        self.drone_coordinates=[18.9998102845, 72.000142461,16.757981]  # Initial approximate drone coordinates
        self.box_type=None                                              # It will store the type wather its "delevery" or "return"
        self.provide_index=0                                            # Used for publishing coordinates in sequence
        self.d_list=None                                                # Storing coordinates from the .csv file
        self.destination_list=[]                                        # Forming the list of appropreat destinations in propper format
        self.coordinates=[]                                             # Buffer for storing coordinate comes from read_and_set_data()
        self.purpose=[]                                                 # List for storing wather its "delevery" or "return"
        self.delivery_grid = [18.9998102845, 72.000142461]              # Coordinate for delevery grid
        self.return_grid = [18.9999367615, 72.000142461]                # Coordinate for return grid
        self.diff_grid = [0.000013552, 0.000014245]                     # Difference between grid points
        self.sample_time = 1                                            # Defining sample time
        self.input_for_csv=[]                                           # For storing all destinations where drone will reach
        self.checkpoint = NavSatFix()                                   # For providing checkpointss to the path_planner script
        self.read_and_set_data()                                        # Calling the function to form sorted checkpoints


        # Publishing
        self.pub_checkpoint = rospy.Publisher('/box_checkpoint', NavSatFix, queue_size=1)
        #subscribing
        rospy.Subscriber('/next_destination_flag',Float32,self.next_destination_callback)
    

        
    #function for the subscription

    def next_destination_callback(self,msg):
        '''
        Purpose:
        ---
        When eDrone will reache at particular destination 'path_planner_beta' node will publish on '/next_destination_flag' topic.
        after getting this flag 'data_publish()' will publish next destination coordinates from this script.

        Input Argument:
        ---
        msg

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the "path_planner_beta" node.
        '''

        if(msg.data==1):
            if(self.provide_index==len(self.coordinates)):
                self.provide_index=len(self.coordinates)
            else:
                self.provide_index+=1



    #mehods for distance measurement

    #below method will convert x coordinates difference in latitude form
    def lat_to_x_diff(self,ip_lat_diff):return (110692.0702932625*ip_lat_diff)

    #below method will convert y coordinates difference in longitude form
    def long_to_y_diff(self,ip_long_diff):return (-105292.0089353767*ip_long_diff)

    def read_and_set_data(self):
        '''
        Purpose:
        ---
        Its main function responcible for sorting the deleveries and returns in perticuler order so that earning will be as max as possibe.
        we have selected far most delevery first to delever and return the parcel boxes.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        called in start for sorting the data and letter data will published by 'data_publish()'.
        '''
        # Boolian list for checking perticuler delevery or return is selected previously or not
        check_list=[]

        # Reading the 'original.csv' file and storing data in appropreat variable
        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'original.csv'), 'r') as x:
            # 'content' is local variable which will store data of .csv file
            content = numpy.array(list(csv.reader(x)))
            # 'box_type' will store status wather its for delevery or return.
            self.box_type = content[:, 0]
            # 'd_list' will store coordinates
            self.d_list = content[:, 1:]

        # Initializing 'check_list' with 'True'
        for i in range(len(self.box_type)):
            check_list.append(True)
        
        #setting maximum threshould
        maxx=1000

        #storing the first coordinates
        buffer_coordinates=self.d_list[0][1].split(';')
        #converting buffer_corrdinates in float for use of comperision
        x,y=float(buffer_coordinates[0]),float(buffer_coordinates[1])

        # Following loop will arrange delevery in propper format
        for Itter in range(int(len(self.box_type)/2)):
            # 'buffer_index' is for traversing in loop
            buffer_index=0

            # Initializing the minimum threshould
            maxx=0

            # 'index' will store the index of preferable coordinates
            index=0
            # Following loop will also arrange the delevery
            for buffer_index in range(len(self.box_type)):
                if(check_list[buffer_index] and self.box_type[buffer_index]=="DELIVERY"):
                    buff_coordinate=self.d_list[buffer_index][1].split(';')
                    x,y=float(buff_coordinate[0]),float(buff_coordinate[1])

                    # 'diff' will store difference between coordinates for delevery
                    diff=math.hypot(self.lat_to_x_diff(abs(self.drone_coordinates[0]-x)),self.long_to_y_diff(abs(self.drone_coordinates[1]-y)))
                    if(diff>=maxx):
                        maxx=diff
                        index=buffer_index
            check_list[index]=False
            self.destination_list.append([self.box_type[index],self.d_list[index][1].split(";"),self.d_list[index][0]]) 
            self.input_for_csv.append([self.box_type[index],self.d_list[index][0],self.d_list[index][1]])

        # Following loop will arrange return from the data of deleveries
        for l in range(int(len(self.box_type)/2)):

            # 'return_index' is traversing in the loop
            rerurn_index=0

            # Defining the madimum threshould for the returns
            r_maxx=1000

            # 'r_index' will store possible return coordinate index
            r_index=0

            # Following loop will also arrange the returns
            for rerurn_index in range(len(self.box_type)):
                if(self.box_type[rerurn_index]=="RETURN"):
                    if(check_list[rerurn_index]):

                        # Below varinable will store coordinate in string type latter helpful for comperision
                        r_buff_coordinate=self.d_list[rerurn_index][0].split(';')

                        # Below variables are just storing one of the coordinate of .csv file and converting it in float type
                        r_x,r_y=float(r_buff_coordinate[0]),float(r_buff_coordinate[1])
                        t_x,t_y=float(self.destination_list[2*l][1][0]),float(self.destination_list[2*l][1][1])

                        # 'r_diff' will store difference between coordinates for returns
                        r_diff=math.hypot(self.lat_to_x_diff(abs(t_x-r_x)),self.long_to_y_diff(abs(t_y-r_y)))
                        if(r_diff<=r_maxx):
                            r_maxx=r_diff
                            r_index=rerurn_index
            if(r_index!=0):
                check_list[r_index]=False

                # Storing data in list for making 'sequenced_manifest_original.csv' and making list of coordinates.
                self.destination_list.insert((2*l+1),[self.box_type[r_index],self.d_list[r_index][0].split(";"),self.d_list[r_index][1]])
                self.input_for_csv.insert((2*l+1),[self.box_type[r_index],self.d_list[r_index][0],self.d_list[r_index][1]])
        
        # Following loop will drefrence the notations like 'A1,X1,etc' into the coordinates in latitude and longitude form 
        for itter in range(len(self.box_type)):
            # Its for switchin from delevery grid to the return grid
            switch_grid_destination=False
            if(self.destination_list[itter][0]=='DELIVERY'):
                if(not switch_grid_destination):
                    # It will store first element of notation
                    box_pose=self.destination_list[itter][2]
                    if(box_pose[0]=='A'):
                        for i in range(4):
                            if(int(box_pose[1])==i):
                                self.coordinates.append([self.delivery_grid[0],((i-1)*self.diff_grid[1]+self.delivery_grid[1]),self.drone_coordinates[2]])
                    elif(box_pose[0]=='B'):
                        for i in range(4):
                            if(int(box_pose[1])==i):
                                self.coordinates.append([(1*self.diff_grid[0]+self.delivery_grid[0]),((i-1)*self.diff_grid[1]+self.delivery_grid[1]),self.drone_coordinates[2]])
                    elif(box_pose[0]=='C'):
                        for i in range(4):
                            if(int(box_pose[1])==i):
                                self.coordinates.append([(2*self.diff_grid[0]+self.delivery_grid[0]),((i-1)*self.diff_grid[1]+self.delivery_grid[1]),self.drone_coordinates[2]])
                    switch_grid_destination=True
                    self.purpose.append(self.destination_list[itter][0])
                if(switch_grid_destination):
                    self.coordinates.append([float(self.destination_list[itter][1][0]),float(self.destination_list[itter][1][1]),float(self.destination_list[itter][1][2])])
                    self.purpose.append(self.destination_list[itter][0])

            elif(self.destination_list[itter][0]=="RETURN"):
                if(not switch_grid_destination):
                    self.coordinates.append([float(self.destination_list[itter][1][0]),float(self.destination_list[itter][1][1]),float(self.destination_list[itter][1][2])])
                    self.purpose.append(self.destination_list[itter][0])

                    switch_grid_destination=True
                if(switch_grid_destination):
                    box_pose=self.destination_list[itter][2]
                    if(box_pose[0]=='X'):
                        for i in range(4):
                            if(int(box_pose[1])==i):
                                self.coordinates.append([self.return_grid[0],((i-1)*self.diff_grid[1]+self.return_grid[1]),self.drone_coordinates[2]])
                    elif(box_pose[0]=='Y'):
                        for i in range(4):
                            if(int(box_pose[1])==i):
                                self.coordinates.append([(1*self.diff_grid[0]+self.return_grid[0]),((i-1)*self.diff_grid[1]+self.return_grid[1]),self.drone_coordinates[2]])
                    elif(box_pose[0]=='Z'):
                        for i in range(4):
                            if(int(box_pose[1])==i):
                                self.coordinates.append([(2*self.diff_grid[0]+self.return_grid[0]),((i-1)*self.diff_grid[1]+self.return_grid[1]),self.drone_coordinates[2]])
                                
                    self.purpose.append(self.destination_list[itter][0])


        # Following loop will form 'sequenced_manifest_original.csv' for making the refrence of delevery or returns to be performed.
        for u in range(len(self.box_type)):
            with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sequenced_manifest_original.csv'), 'a') as sequenced_csv:
                sequence_writer = csv.writer(sequenced_csv, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                sequence_writer.writerow([self.input_for_csv[u][0],self.input_for_csv[u][1],self.input_for_csv[u][2]])

                    
        
    def data_publish(self):
        '''
        Purpose:
        ---
        It will piblish coordinates continuously which will being received by the 'path_planner_beta' node.

        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        it will being called continuously as per particular frequency

        data_publish()
        '''
        [self.checkpoint.latitude,self.checkpoint.longitude,self.checkpoint.altitude]=self.coordinates[self.provide_index]
        self.checkpoint.header.frame_id=self.purpose[self.provide_index]
        self.pub_checkpoint.publish(self.checkpoint)
        
# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the Dara_processing class's function data_publish() to publish perticuler coordinates.
if __name__ == "__main__":
    reader = Data_processing()
    rate = rospy.Rate(1/reader.sample_time)
    while not rospy.is_shutdown():
        reader.data_publish()
        rate.sleep()