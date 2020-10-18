#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

counter = 0
counter1 = 0
x, x0, y, y0 = 0, 0, 0, 0
error = 0.01


def initialCall(Pose):
    global x, y, x0, y0, counter
    if counter == 0:
        counter = 1
        x0, y0 = Pose.x, Pose.y
    x, y = Pose.x, Pose.y


def main():
    rospy.init_node("revolve", anonymous=True)
    v_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    v_msg = Twist()

    t0 = rospy.Time.now().to_sec()
    print("Lets move our Bot")

    v_msg.linear.x = 2
    v_msg.linear.y = 0
    v_msg.linear.z = 0
    v_msg.angular.x = 0
    v_msg.angular.y = 0
    v_msg.angular.z = 1
    v_pub.publish(v_msg)
    rospy.Subscriber('turtle1/pose', Pose, initialCall)

    global counter1
    while(True):
        rospy.Rate(15).sleep()
        t1 = rospy.Time.now().to_sec()
        v_pub.publish(v_msg)
        rad = v_msg.angular.z * (t1-t0)
        rospy.loginfo("Moving in Circle\n%f", rad)

        global counter1
        if(counter1 == 0):
            while int(y) == int(y0):
                v_pub.publish(v_msg)
                continue
            counter1 = counter1+1
            continue

        if(y <= y0+error):
            v_msg.linear.x = 0
            v_pub.publish(v_msg)
            break

    rospy.loginfo("Goal Reached")
    while not rospy.is_shutdown():
        v_pub.publish(v_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
