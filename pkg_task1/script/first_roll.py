#!/usr/bin/env python
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf
rospy.init_node('attitude_controller', anonymous=True)
drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
drone_orientation_euler = [0.0, 0.0, 0.0]
sample_time = 0.060


drone_orientation_quaternion = [0, 0, 0, 0]


def imu_callback(msg):

    drone_orientation_quaternion[0] = msg.orientation.x
    drone_orientation_quaternion[1] = msg.orientation.y
    drone_orientation_quaternion[2] = msg.orientation.z
    drone_orientation_quaternion[3] = msg.orientation.w


(drone_orientation_euler[0], drone_orientation_euler[1], drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
    [drone_orientation_quaternion[0], drone_orientation_quaternion[1], drone_orientation_quaternion[2], drone_orientation_quaternion[3]])
rospy.Subscriber('/edrone/imu/data', Imu, imu_callback)
while(True):
    print(drone_orientation_euler)
    rospy.Rate(100).sleep()
