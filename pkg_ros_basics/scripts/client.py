#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import String
i = None


def call_back(S):
    global i
    i = S.data
    print(i)


def client(x):
    rospy.wait_for_service('/edrone/activate_gripper')
    pover = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    reasult = pover(x)
    return reasult.result


def main():
    rospy.init_node('pose')
    rospy.Subscriber('/edrone/gripper_check', String, call_back)
    # print(i)
    if(i == 'True'):
        print(client(bool(1)))
    else:
        print(client(bool(0)))

    # x = i
    # print(client(i))
if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()

    # x = bool(0)
    # print(client(x))
