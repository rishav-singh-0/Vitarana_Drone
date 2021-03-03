#!/usr/bin/env python
'''
#Team ID:            0983
#Theme:              VITARANA DRONE
#Author List:        Rishav Singh,Kashyap Joshi
#Filename:           gripper_client.py
#Functions:          gripper_check_callback,gripper_client,grip_check
#Global Variables:   None
'''

'''
This python file runs a ROS-node of name 'marker_detect' which will calculate difference of cross marker from the eDrone.
This node publishes and subsribes the following topics:
        SUBSCRIPTIONS                        PUBLICATIONS                 
        /gripp_flag                          None

'''
from vitarana_drone.srv import *
from vitarana_drone.msg import *
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
import rospy


class Grip():
    '''
    Purpose:
    ---
    This class will contain all the member functions and defined variable to control attechment and detechment of parcel box.

    Input Argument:
    ---
    None
    '''

    def __init__(self):
        '''
        it will initialize all the variables and define subscriptinos and publications.
        
        '''

        rospy.init_node('gripper_client')           # initializing a node
        # necessary variables
        self.attech_situation = -1                  # check if box is attechable
        self.attech_constraint = 0                  # limits the service for atteching request
        self.detech_constraint = 0                  # limits the service for deteching request


        #Subcribe
        rospy.Subscriber('/gripp_flag', String, self.gripper_check_callback)


    def gripper_check_callback(self, state):
        '''
        Purpose:
        ---
        Its callback function which will makesure that if box is attechable or not.
        if attachable it will save 'True'.
        if box is not attachable then it will save 'False' in self.attech_situation variable

        Input Argument:
        ---
        data

        Returns:
        ---
        None

        Example call:
        ---
        Called automatically when appropreat data will being published by the "path_planner_beta" node.
        '''
        # print(state.data, bool(state.data))
        # print(state.data)
        self.attech_situation = state.data

    def gripper_client(self, check_condition):

        '''
        Purpose:
        ---
        Basically this function is made for service call and it will wait for service to active.
        when service will active it will send request to grip or drop box and receive appropreat feedback from the service.
        
        Input Argument:
        ---
        check_condition

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

    def grip_check(self):

        '''
        Purpose:
        ---
        this function will determines wather to attech or detech box by calling the gripper service.
        it will only access that to attech or detech information.

        
        Input Argument:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        Every time this function is called for attaching or deteching the box.

        grip.grip_check()

        '''

        #this condition is for the attaching the box
        if(self.attech_situation=='True' and self.attech_constraint == 0):
            # while( self.gripper_client(True)==False):
            #     self.gripper_client(True)
            self.attech_constraint += 1
        #this condition is for the deteching the box
        elif(self.attech_situation=='False' and self.detech_constraint == 0):
            self.gripper_client(False)
            self.detech_constraint += 1
        #this condition is for reinitializing the constrain variable because it will not cross 1 in the iplimentation of condition.
        if(self.attech_constraint and self.detech_constraint==1):
            self.detech_constraint=self.attech_constraint=0

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the Grip class's function grip_check to grip or ungrip the box.

if __name__ == "__main__":
    grip = Grip()
    while not rospy.is_shutdown():
        grip.grip_check()
