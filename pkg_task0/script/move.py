#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.141592654


def move():
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's move your robot")

    vel_msg.linear.x = 5
    vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
    vel_msg.angular.z = 2

    # initial time which will useful for calculating the distance
    t0 = rospy.Time.now().to_sec()
    cdd = 0

    # let's revolve the turtle
    # it will stop after one revolution
    def distance(v, w): return 2*PI * (v**2 + w**2)**0.5/w
    while(cdd < distance(vel_msg.linear.x, vel_msg.angular.z)):
        # publishing the velocity to the turtle node
        velocity_publisher.publish(vel_msg)
        # taking the second time for claculating the distance
        t1 = rospy.Time.now().to_sec()
        # calculating distance
        cdd = vel_msg.linear.x*(t1-t0)
        rospy.loginfo("moving in circle\n %f", cdd)
        # turtle bot will stop moving after this loop
    vel_msg.linear.x = 0  # makin linear velocity 0 for stopping the bot
    rospy.loginfo("goal reached")
    # its for user convineance
    while not rospy.is_shutdown():
        # it will publish the msg of velocity
        velocity_publisher.publish(vel_msg)
        continue


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
