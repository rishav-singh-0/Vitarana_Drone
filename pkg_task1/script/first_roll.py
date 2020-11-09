#!/usr/bin/env python
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import tf
import math

rospy.init_node('attitude_controller', anonymous=True)
drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
drone_orientation_euler = [0, 0, 0]
sample_time = 0.060


def imu_callback(msg):

    drone_orientation_quaternion[0] = msg.orientation.x
    drone_orientation_quaternion[1] = msg.orientation.y
    drone_orientation_quaternion[2] = msg.orientation.z
    drone_orientation_quaternion[3] = msg.orientation.w


def pid(PidTune):
    global kp, kd, ki
    kp = PidTune.Kp/5
    kd = PidTune.Kd
    ki = PidTune.Ki/1


def prop(msg):
    global pwm_msg
    pwm_msg.prop1 = msg.prop_speed.prop1
    pwm_msg.prop2 = msg.prop_speed.prop2
    pwm_msg.prop3 = msg.prop_speed.prop3
    pwm_msg.prop4 = msg.prop_speed.prop4


pwm = 512.0
desired = 3

ep = dev = ei = error = input_ctrl_signal = 0
kp = kd = ki = 0


# Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
rospy.Subscriber('/edrone/imu/data', Imu, imu_callback)

pwm_msg = prop_speed()
pwm_msg.prop1 = pwm_msg.prop2 = pwm_msg.prop3 = pwm_msg.prop4 = 0
roll_error = Float32()
pitch_error = Float32()
yaw_error = Float32()


while(True):
    (drone_orientation_euler[0], drone_orientation_euler[1], drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
        [drone_orientation_quaternion[0], drone_orientation_quaternion[1], drone_orientation_quaternion[2], drone_orientation_quaternion[3]])
    for i in range(3):
        drone_orientation_euler[i] = math.degrees(drone_orientation_euler[i])

    rospy.Subscriber("/pid_tuning", PidTune, pid)
    rospy.Subscriber('/drone_command', prop_speed, prop)
    error = desired - drone_orientation_euler[0]
    dt = 0.01
    dev = (error-ep)/dt
    ep = error
    ei = ei + error*dt
    output = kp*error + ki*ei + kd*dev

    pwm_msg.prop1 = pwm_msg.prop2 = pwm_msg.prop3 = pwm_msg.prop4 = output
    print(output, drone_orientation_euler[0])

    roll_error = pitch_error = yaw_error = error
    pwm_pub.publish(pwm_msg)
    roll_pub.publish(roll_error)
    roll_pub.publish(pitch_error)
    roll_pub.publish(yaw_error)
    rospy.Rate(100).sleep()
