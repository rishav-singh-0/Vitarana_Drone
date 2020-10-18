#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

PI = 3.141592


def initialCall(Pose):
    x0, y0 = Pose.x, Pose.y
    print(x0, y0)


# def check():
#     if()


def main():
    rospy.init_node("revolve", anonymous=True)
    v_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    v_msg = Twist()

    print("Lets move out Bot")

    v_msg.linear.x = 5
    v_msg.linear.y = 0
    v_msg.linear.z = 0
    v_msg.angular.x = 0
    v_msg.angular.y = 0
    v_msg.angular.z = 2

    rospy.Subscriber('turtle1/pose', Pose, initialCall)
    v_pub.publish(v_msg)
    # while(check):

    while not rospy.is_shutdown():
        v_pub.publish(v_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
