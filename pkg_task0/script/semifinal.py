#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time 
y=0#it is defined as global variable to take the theta from the 'turtle1/pose' topic
speed=5#it is linear velocity of the x 
PI = 3.141592654
e=0
q=0
RA=2.18*PI#calculating required angle in radian
##its expected theeta figured out from rostopic "echo /turtle1/pose"  
#this is call back function for subcribing 'turtle1/pose'
f=0.2
radius=1
x=0
varx=0
vary=0
def printer(Pose):
    global y
    global x
    global e
    global varx
    global vary
    while (Pose.x !=0 and Pose.y !=0 and e==0):
        varx=Pose.x
        vary=Pose.y
        e=e+1
    y=Pose.y
    x=Pose.x
    #print(x,"and",y)
#r =rospy.Rate(10)    
#its main working function for moving the turtle bot
def move():
    rospy.Subscriber('turtle1/pose',Pose,printer)#Subcribing the 'turtle1/pose' topic
    #print(x,"and",y)
    rospy.init_node('mover',anonymous=True)#making the node for communication
    velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)#its a publishing functios which will cofigure the '/turtle1/cmd_vel' and also queue size
    t0 = rospy.Time.now().to_sec()#inital time
    current_distance = 0#inial measured distance
    while (True):
        vel_msg = Twist()
        #assigning the velocities
        vel_msg.linear.x = 5
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
        vel_msg.angular.z =2
        velocity_publisher.publish(vel_msg)#publishing the velocities
        rospy.Subscriber('turtle1/pose',Pose,printer)#Subcribing the 'turtle1/pose' topic

        t1 = rospy.Time.now().to_sec()#current time
        current_distance = speed*(t1-t0)#calculation of distance

        #rospy.loginfo("Moving in Circle\n%f",(vel_msg.angular.z*(t1-t0)))#printing the radian measurement
        #below block is for stopping the turtle
       # print(varx)

        
        global q
        def distance(v, w): return 2*PI * (v**2 + w**2)**0.5/w
        print(x,"and",y)
        if(q==0):
            while(((round(y,0)==round(vary,0)))):
                vel_msg.linear.x = 5
                vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
                vel_msg.angular.z =2
                velocity_publisher.publish(vel_msg)
                continue
            q=q+1

        elif((q==1)and(round(y,1)==round(vary,1))):
            # velocity_publisher.publish(vel_msg)
            print(x,"and",y)
            # print("you are great")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 2
            velocity_publisher.publish(vel_msg)
            break

        # else:
        #     vel_msg.linear.x = 0
        #     vel_msg.angular.z = 2
        #     velocity_publisher.publish(vel_msg)
        #     break
        #rospy.sleep(f)
    rospy.loginfo("goal reached")
    #waiting for user to terminate the program
    while not rospy.is_shutdown():
        vel_msg.angular.z =2
        velocity_publisher.publish(vel_msg)
        continue
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass