#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32


class Grip():
    def __init__(self):
        rospy.init_node('gripper_client')

        self.attech_situation = False
        self.detech_req = False

        rospy.Subscriber('/edrone/gripper_check', String, self.call_back)
        rospy.Subscriber('op_flag', Float32, self.detech_msg)

    def call_back(self, state):
        self.attech_situation = state.data

    def gripper_client(self, check_condition):
        rospy.wait_for_service('/edrone/activate_gripper')
        carry = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        msg_container = carry(check_condition)
        return msg_container.result

    def detech_msg(self, msg):
        self.detech_req = msg.data
        # print(self.detech_req)

    def grip_check(self):
        if(self.attech_situation == 'True' and self.detech_req == 0):
            self.gripper_client(True)
        elif(self.detech_req == 1):
            self.gripper_client(False)


if __name__ == "__main__":
    grip = Grip()
    while not rospy.is_shutdown():
        grip.grip_check()
