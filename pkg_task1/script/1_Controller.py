#!/usr/bin/env python

# importing required libraries
import rospy
from vitarana_drone.msg import prop_speed


def main():
    # initialising node
    rospy.init_node("controller", anonymous=True)
    ps_pub = rospy.Publisher(
        "/vitarana_drone/prop_speed", prop_speed, queue_size=10)

    print("Lets move our Bot")
    ps_msg = prop_speed()
    ps_msg.prop1 = 512.0
    ps_msg.prop2 = 512.0
    ps_msg.prop3 = 512.0
    ps_msg.prop4 = 512.0

    ps_pub.publish(ps_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
