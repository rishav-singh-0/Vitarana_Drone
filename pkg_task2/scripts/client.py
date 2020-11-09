#!/usr/bin/env python
from vitarana_drone.srv import *
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
attech_situation, detech_req = None, None


def call_back(S):
    global attech_situation
    attech_situation = S.data


def client(x):
    rospy.wait_for_service('/edrone/activate_gripper')
    pover = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    reasult = pover(x)
    return reasult.result


def detech_msg(msg):


<< << << < HEAD
   global j
    j = msg.data


w = 0
== == == =
   global detech_req
    detech_req = msg.data
    print(detech_req)
>>>>>> > 1bd2250... further bugs are fixed


def main():
    rospy.init_node('pose')
    rospy.Subscriber('/edrone/gripper_check', String, call_back)
    rospy.Subscriber('op_flag', Float32, detech_msg)

    # print(i)
<< << << < HEAD
   if(i == 'True'):
        print(client(bool(1)))
    elif(j == 1):
        print(client(bool(0)))
    elif(w == 0):
        print(client(bool(0)))
        w += 1
== =====
   if(attech_situation == 'True' and detech_req == 0):
        client(bool(1))
    elif(detech_req >= 1 and attech_situation == 'True'):
        client(bool(0))
>>>>>> > 1bd2250... further bugs are fixed


if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()
