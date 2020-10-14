#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI=3.141592654
def move():
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's move your robot")
    
    vel_msg.linear.x =5
    vel_msg.linear.y =vel_msg.linear.z=vel_msg.angular.x=vel_msg.angular.y=0
    vel_msg.angular.z = 2

    t0 = rospy.Time.now().to_sec()#it's for initial time which will useful fir calculating the distance
    cdd = 0

    #let'srevolve the turtle  
    #it will stop after one revolution  
    while(cdd <=2*PI*(((vel_msg.angular.z**2)+(vel_msg.linear.x**2))**0.5)/vel_msg.angular.z):
            #publishing the velocity to the turtle node
        velocity_publisher.publish(vel_msg)
            #taking the second time for claculating the distance
        t1=rospy.Time.now().to_sec()
            #calculating distance
        cdd= vel_msg.linear.x*(t1-t0)
        rospy.loginfo("moving in circle\n %f",cdd)
        #rospy.loginfo(cdd)
        #turtle bot will stop moving after this loop
    vel_msg.linear.x = 0#makin linear velocity 0 for stopping the bot
    rospy.loginfo("goal reached")
        #it will publish the msg of velocity
    velocity_publisher.publish(vel_msg)
        #its for user convineance
    while not rospy.is_shutdown():  
        continue

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
