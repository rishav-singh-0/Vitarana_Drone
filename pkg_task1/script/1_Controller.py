#!/usr/bin/env python

# importing required libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def main():
    # initialising node
    rospy.init_node("1_controller", anonymous=True)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
