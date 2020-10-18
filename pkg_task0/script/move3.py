#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
j=0
k=0
y=0
t=0
PI = 3.141592654
x=5
z=2
def printer(Pose):
    global j
    global k
    global y
    global t
    global q
    j=Pose.x
    k=Pose.angular_velocity
    y=Pose.linear_velocity
    q=Pose.theta
    #print(Pose.x)
def move():
    rospy.Subscriber('turtle1/pose',Pose,printer)
    e=0
        
    rospy.init_node('mover',anonymous=True)
    velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        #print(j,"and",k)
    print(k,"and",y)
    vel_msg = Twist()
    speed = 5
    vel_msg.linear.x = x
    vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
    vel_msg.angular.z = z
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    def distance(v, w): return 2*PI * (v**2 + w**2)**0.5/w
    while(((current_distance < distance(vel_msg.linear.x, vel_msg.angular.z)))):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed*(t1-t0)
        rospy.loginfo("Moving in Circle")
        print(current_distance)
    vel_msg.linear.x = 0  # makin linear velocity 0 for stopping the bot
    vel_msg.angular.z = 0
    print("the angle is",q)
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("goal reached")
    while not rospy.is_shutdown():
        continue
        
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass