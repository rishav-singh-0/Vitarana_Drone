#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

class Grip():
    def __init__(self):
        rospy.init_node('gripper_client')


        self.gps_position=[0,0,0]
        self.final_destination=[0,0,0]
        self.container=[0,0,0]
        self.attech_situation = False
        self.cnt=0
        self.cnt1=0
        self.req=False

        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_destination_callback)
    

    def final_destination_callback(self, msg):
        self.container= [msg.latitude, msg.longitude, msg.altitude]
        if(self.final_destination!=self.container):
            self.final_destination = self.container

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def gripper_check_callback(self, state):
        self.attech_situation = state.data

    def gripper_client(self, check_condition):
        rospy.wait_for_service('/edrone/activate_gripper')
        carry = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        msg_container = carry(check_condition)
        return msg_container.result
    


    def grip_check(self):
        if(self.final_destination!=self.container):
            if(-0.00001517<(self.final_destination[0]-self.gps_position[0])<0.00001517):
                if(-0.15 <= (self.final_destination[2]-self.gps_position[2]) <= 0.15):
                    print(self.final_destination)
                    self.req=not self.req
                

        # print(self.attech_situation)
        # print(self.cnt)
        if(self.attech_situation=='True' and self.cnt==0):
            if(self.req):
                self.gripper_client(True)
                self.cnt+=1
            #if(self.detech==1):
            if( self.req==False and self.cnt==1):
                self.gripper_client(False)


if __name__ == "__main__":
    grip = Grip()
    while not rospy.is_shutdown():
        grip.grip_check()
