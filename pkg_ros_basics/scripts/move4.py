#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
y = 0  # it is defined as global variable to take the theta from the 'turtle1/pose' topic
speed = 5  # it is linear velocity of the x
z = 0
e = 0
f = 0
x = 0
PI = 3.141592654
RA = 2.18*PI  # calculating required angle in radian
# its expected theeta figured out from rostopic "echo /turtle1/pose"
expected_theta = 0.08481469750404358
# this is call back function for subcribing 'turtle1/pose'


def printer(Pose):
    global x
    global e
    global y
    y = Pose.y
    x = Pose.y


# its main working function for moving the turtle bot
def move():
    global f
    # making the node for communication
    rospy.init_node('mover', anonymous=True)
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)  # its a publishing functios which will cofigure the '/turtle1/cmd_vel' and also queue size
    t0 = rospy.Time.now().to_sec()  # inital time
    current_distance = 0  # inial measured distance
    while (True):

        vel_msg = Twist()
        # assigning the velocities
        vel_msg.linear.x = 1
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
        vel_msg.angular.z = 1
        velocity_publisher.publish(vel_msg)  # publishing the velocities
        # Subcribing the 'turtle1/pose' topic
        rospy.Subscriber('turtle1/pose', Pose, printer)
        t1 = rospy.Time.now().to_sec()  # current time
        current_distance = speed*(t1-t0)  # calculation of distance

        # printing the radian measurement
        rospy.loginfo("Moving in Circle\n%f", (1*(t1-t0)))
        # below block is for stopping the turtle
        def distance(v, w): return 2*PI * (v**2 + w**2)**0.5/w
        # time.sleep(1)
        print(y)
        if((round(x, 9) == 5.544444561) and (round(y, 9) == 5.544444561)):
            break
            # if(f==0):
            #     f=f+1
            # else:
            #     vel_msg.linear.x = 0
            #     vel_msg.angular.z = 2
            #     velocity_publisher.publish(vel_msg)
            #     break
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
