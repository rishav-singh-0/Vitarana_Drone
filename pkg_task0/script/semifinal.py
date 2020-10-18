#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
e = 0
q = 0


def printer(Pose):
    global y, x, e, varx, vary
    while (Pose.x != 0 and Pose.y != 0 and e == 0):
        varx = Pose.x
        vary = Pose.y
        e = e+1
    y = Pose.y
    x = Pose.x


def move():
    rospy.Subscriber('turtle1/pose', Pose, printer)
    rospy.init_node('mover', anonymous=True)
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)  # its a publishing functios which will cofigure the '/turtle1/cmd_vel' and also queue size
    t0 = rospy.Time.now().to_sec()  # inital time
    while (True):
        vel_msg = Twist()
        vel_msg.linear.x = 5
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
        vel_msg.angular.z = 2
        velocity_publisher.publish(vel_msg)  # publishing the velocities
        rospy.Subscriber('turtle1/pose', Pose, printer)

        t1 = rospy.Time.now().to_sec()  # current time

        global q
        if(q == 0):
            while int(y) == int(vary):
                velocity_publisher.publish(vel_msg)
                continue
            q = q+1
            continue

        if (round(y, 1) == round(vary, 1)):
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            break

    rospy.loginfo("goal reached")
    # waiting for user to terminate the program
    while not rospy.is_shutdown():
        vel_msg.angular.z = 2
        velocity_publisher.publish(vel_msg)
        continue


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
