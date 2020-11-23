#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import rospy

class Grip():
    def __init__(self):
        rospy.init_node('gripper_client')#initializing a node
        #necessary variables
        self.gps_position = [0, 0, 0]
        self.final_destination = [0, 0, 0]
        self.container = [0, 0, 0]
        self.attech_situation = False#check if box is attechable
        self.attech_constraint = 0#limits the service for atteching request
        self.reading_constraint = 0#limits the process reletive to the number boxes
        self.detech_constraint = 0#limits the service for deteching request
        self.attech_detech_switch = False#determines when to attech box and when to detech the box

        rospy.Subscriber('/edrone/gripper_check', String,
                         self.gripper_check_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/final_setpoint', NavSatFix,
                         self.final_destination_callback)

    def final_destination_callback(self, msg):
        '''function will take final destinations from qr_detect.py script'''
        self.container = [msg.latitude, msg.longitude, msg.altitude]
        if(self.final_destination != self.container):
            self.final_destination = self.container
            self.reading_constraint += 1

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def gripper_check_callback(self, state):
        self.attech_situation = state.data

    def gripper_client(self, check_condition):
        '''this function will call and wait for the gripper service'''
        rospy.wait_for_service('/edrone/activate_gripper')
        carry = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        msg_container = carry(check_condition)
        return msg_container.result#true if box is atteched and visa versa

    def grip_check(self):
        '''this function will handle attech and detech service'''
        if(-0.00001517 < (self.final_destination[0]-self.gps_position[0]) < 0.00001517):
            if(-0.15 <= (self.final_destination[2]-self.gps_position[2]) <= 0.15):
                if(self.reading_constraint % 2 != 0):
                    self.attech_detech_switch = True
                else:
                    self.attech_detech_switch = False

        if(self.attech_situation == 'True'):
            if(self.attech_detech_switch and self.attech_constraint == 0):
                self.gripper_client(True)
                self.attech_constraint += 1
            if(self.attech_detech_switch == False and self.detech_constraint == 0):
                self.gripper_client(False)
                self.detech_constraint += 1

if __name__ == "__main__":
    grip = Grip()
    while not rospy.is_shutdown():
        grip.grip_check()
