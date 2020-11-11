#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
attech_situation, detech_req = None, None


class Grip():
    def __init__(self):
        rospy.init_node('gripper_client')

        self.attech_situation = False
        self.detech_msg = False
        self.detech_req = False

        rospy.Subscriber('/edrone/gripper_check', String, self.call_back)
        rospy.Subscriber('op_flag', Float32, self.detech_msg)

    def call_back(self, state):
        self.attech_situation = state.data

    def client(self, x):
        rospy.wait_for_service('/edrone/activate_gripper')
        pover = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        reasult = pover(x)
        return reasult.result

    def detech_msg(self, msg):
        self.detech_req = msg.data
        # print(self.detech_req)

    def main(self):
        # print(i)
        if(self.attech_situation == 'True' and self.detech_req == 0):
            self.client(bool(1))
        elif(self.detech_req >= 1 and self.attech_situation == 'True'):
            self.client(bool(0))


if __name__ == "__main__":
    grip = Grip()
    while not rospy.is_shutdown():
        grip.main()
