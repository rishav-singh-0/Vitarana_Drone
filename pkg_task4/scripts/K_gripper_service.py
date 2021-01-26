#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
from std_msgs.msg import String,Float32
from sensor_msgs.msg import NavSatFix
import rospy


class Grip():
    def __init__(self):
        rospy.init_node('gripper_client')   # initializing a node
        # necessary variables
        self.gps_position = [0, 0, 0]
        self.final_destination = [0, 0, 0]
        self.container = [0, 0, 0]
        self.attech_situation = -1       # check if box is attechable
        self.attech_constraint = 0          # limits the service for atteching request
        # limits the process reletive to the number boxes
        self.detech_constraint = 0          # limits the service for deteching request
        # determines when to attech box and when to detech the box

        rospy.Subscriber('/gripp_flag', String,
                         self.gripper_check_callback)
        

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def gripper_check_callback(self, state):
        print(state.data)
        self.attech_situation = state.data

    def gripper_client(self, check_condition):
        '''this function will call and wait for the gripper service'''

        rospy.wait_for_service('/edrone/activate_gripper')
        carry = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        msg_container = carry(check_condition)
        return msg_container.result     # true if box is atteched and visa versa

    def grip_check(self):
        '''this function will handle attech and detech service'''

        if(self.attech_situation == 'True' and self.attech_constraint == 0):
            self.gripper_client(True)
            self.attech_constraint += 1
        elif(self.attech_situation == 'False' and self.detech_constraint == 0):
            self.gripper_client(False)
            self.detech_constraint += 1

        if(self.attech_constraint==1 and self.detech_constraint==1):
            self.detech_constraint=self.attech_constraint=1

if __name__ == "__main__":
    grip = Grip()
    while not rospy.is_shutdown():
        grip.grip_check()
