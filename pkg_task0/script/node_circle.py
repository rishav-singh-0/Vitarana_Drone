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
    speed = 5

    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 2.5

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    def distance(v, w): return 2*PI * (v**2 + w**2)**0.5/w
    while(current_distance < distance(vel_msg.linear.x, vel_msg.angular.z)):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed*(t1-t0)
        rospy.loginfo("Moving in Circle")
        print(current_distance)
    vel_msg.linear.x = 0  # makin linear velocity 0 for stopping the bot
    velocity_publisher.publish(vel_msg)
    while not rospy.is_shutdown():
        continue


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
