#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
j = 0
k = 0
y = 0
t = 0
PI = 3.141592654


def printer(Pose):
    global j
    global k
    global y
    global t
    j = Pose.x
    t = Pose.x
    y = Pose.theta
    # y=Pose.linear_velocity
    # print(Pose.x)


def move():

    e = 0
    while (True):
        rospy.Subscriber('turtle1/pose', Pose, printer)
        rospy.init_node('mover', anonymous=True)
        velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        # print(j,"and",k)
        # print(k,"and",y)
        vel_msg = Twist()
        vel_msg.linear.x = 5
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
        vel_msg.angular.z = 2
        velocity_publisher.publish(vel_msg)

        if(y == 0.08481469750404358):
            # if(e==0):
            #     e=e+1
            #     print(e)
            # else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 2
            velocity_publisher.publish(vel_msg)
            break


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
