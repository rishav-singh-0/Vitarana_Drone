#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
i = None
j = None


def call_back(S):
    global i
    i = S.data
    # print(i)


def client(x):
    rospy.wait_for_service('/edrone/activate_gripper')
    pover = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    reasult = pover(x)
    return reasult.result


def detech_msg(msg):
    global j
    j = msg.data
    print(j)


w = 0


def main():
    global w
    rospy.init_node('pose')
    rospy.Subscriber('/edrone/gripper_check', String, call_back)
    rospy.Subscriber('op_flag', Float32, detech_msg)
    # print(i)
    if(i == 'True' and j == 0):
        client(bool(1))
    elif(j == 1 and i == 'True'):
        client(bool(0))
    # elif(w == 0):
    #     client(bool(0))
    #     w += 1

    # x = i
    # print(client(i))
if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()

    # x = bool(0)
    # print(client(x))
