#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time 
y=0#it is defined as global variable to take the theta from the 'turtle1/pose' topic
speed=5#it is linear velocity of the x 
PI = 3.141592654
e=0
RA=2.18*PI#calculating required angle in radian
expected_theta=0.08481469750404358#its expected theeta figured out from rostopic "echo /turtle1/pose"  
#this is call back function for subcribing 'turtle1/pose'
f=0.09
radius=1
def printer(Pose):
    global y
    y=Pose.theta
#r =rospy.Rate(10)    
#its main working function for moving the turtle bot
def move():
    rospy.init_node('mover',anonymous=True)#making the node for communication
    velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)#its a publishing functios which will cofigure the '/turtle1/cmd_vel' and also queue size
    t0 = rospy.Time.now().to_sec()#inital time
    current_distance = 0#inial measured distance
    while (True):
        rospy.Subscriber('turtle1/pose',Pose,printer)#Subcribing the 'turtle1/pose' topic
        vel_msg = Twist()
        #assigning the velocities
        vel_msg.linear.x = radius*2*PI*f
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
        vel_msg.angular.z =2*PI*f
        velocity_publisher.publish(vel_msg)#publishing the velocities

        t1 = rospy.Time.now().to_sec()#current time
        current_distance = speed*(t1-t0)#calculation of distance

        rospy.loginfo("Moving in Circle\n%f",(vel_msg.angular.z*(t1-t0)))#printing the radian measurement
        #below block is for stopping the turtle
        global e
        def distance(v, w): return 2*PI * (v**2 + w**2)**0.5/w
        if(e==0):
            while(round(y,1)==0.0):
                vel_msg.linear.x = 2*2*PI*f
                vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
                vel_msg.angular.z =2*PI*f
                velocity_publisher.publish(vel_msg)
                #print("1")
                continue
            e=e+1
        elif((e==1)and(round(y,1)!=0.0)):
            velocity_publisher.publish(vel_msg)
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            break
        rospy.sleep(f)
    rospy.loginfo("goal reached")
    #waiting for user to terminate the program
    while not rospy.is_shutdown():
        vel_msg.angular.z =2*PI*0.2
        velocity_publisher.publish(vel_msg)
        continue
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass