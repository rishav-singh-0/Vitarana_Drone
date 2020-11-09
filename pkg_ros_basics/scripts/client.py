#!/usr/bin/env python
from vitarana_drone.srv import *
import rospy


def client(x):
    rospy.wait_for_service('/edrone/activate_gripper')

    pover = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    reasult = pover(x)
    return reasult.result


if __name__ == "__main__":
    x = bool(1)
    print(client(x))
