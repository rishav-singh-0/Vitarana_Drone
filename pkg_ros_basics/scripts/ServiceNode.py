#!/usr/bin/env python

from pkg_ros_basics.srv import *
import rospy


def call_back(msg):
    print(msg.A, msg.B, "\n", msg.A**msg.B)
    return exampleResponse(msg.A**msg.B)


def check_srv():
    rospy.init_node("server", anonymous=True)
    srv = rospy.Service('power', example, call_back)

    rospy.spin()


if __name__ == "__main__":
    check_srv()
