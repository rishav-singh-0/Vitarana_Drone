#!/usr/bin/env python

# importing required libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# initialising required variables for safer side
counter = 0
counter1 = 0
x, x0, y, y0 = 0, 0, 0, 0
error = 0.01


def initialCall(Pose):
    '''Function is called when v_sub is subscribed to turtle1/pose'''
    global x, y, x0, y0, counter
    if counter == 0:
        # storing initial coordinates in x0 and y0
        counter = 1
        x0, y0 = Pose.x, Pose.y
    # Current coordinates stored in x and y
    x, y = Pose.x, Pose.y


def main():
    # initialising node
    rospy.init_node("node_revolve", anonymous=True)
    # Publishing Twist message through v_pub
    v_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # Storing Twist message in v_msg
    v_msg = Twist()

    # storing initial time in t0
    t0 = rospy.Time.now().to_sec()
    print("Lets move our Bot")

    # initialising v_msg to give proper moving state
    v_msg.linear.x = 2
    v_msg.linear.y = 0
    v_msg.linear.z = 0
    v_msg.angular.x = 0
    v_msg.angular.y = 0
    v_msg.angular.z = 1

    # publishing the initial v_msg that is speed parameters of turtle bot
    v_pub.publish(v_msg)

    # Subscribing turtle/pose and running initialCall on subscribing
    rospy.Subscriber('turtle1/pose', Pose, initialCall)

    global counter1
    while(True):
        # defining the program rate time for publishing and subscribing
        rospy.Rate(15).sleep()
        t1 = rospy.Time.now().to_sec()
        # t1 is to store current time

        # publishing the initial v_msg that is speed parameters of turtle bot
        v_pub.publish(v_msg)
        rad = v_msg.angular.z * (t1-t0)  # Calculating Radian angle covered
        rospy.loginfo("Moving in Circle\n%f", rad)

        global counter1
        if(counter1 == 0):
            # removing unwanted/repeated readings
            while int(y) == int(y0):
                v_pub.publish(v_msg)
                continue
            counter1 = counter1+1
            continue

        # Stopping the bot when the condition is satisfied
        if(y <= y0+error):
            v_msg.linear.x = 0
            v_pub.publish(v_msg)
            break

    # Verifying if the bot has reached its final destination
    rospy.loginfo("Goal Reached")
    while not rospy.is_shutdown():
        v_pub.publish(v_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
