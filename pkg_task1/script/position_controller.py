#!/usr/bin/env python

# importing required libraries

import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix


class Command():
    '''Navigating the Drone through commands'''

    def __init__(self):
        rospy.init_node('position_controller')

        # initialising gps position
        # [latitude,longitude,altitude]
        self.gps_position = [0, 0, 0]

        # initialising desired position
        # [latitude,longitude,altitude]
        self.destination1 = [0, 0, 3]

        # since there is single thing to tune here, we can make it shorter by listing
        # [kp, ki, kd]
        self.kcontrol = [0, 0, 0]

        self.kp = 41.6
        self.ki = 0
        self.kd = 75

        self.ei = 0
        self.dev = 0

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.prev_error = 0
        self.error = 0
        # Taking max value of prop speed 1000 not 1024 cause in real world it is impossible to operate motors on its maximum speed
        self.max_values = [1000, 1000, 1000, 1000]
        self.min_values = [0, 0, 0, 0]

        self.sample_time = 0.060

        # Publishers
        self.cmd_pub = rospy.Publisher(
            '/drone_command', edrone_cmd, queue_size=1)

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_throttle',
                         PidTune, self.throttle_set_pid)

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def throttle_set_pid(self, msg):
        self.kcontrol[0] = throttle.Kp * 0.06
        self.kcontrol[1] = throttle.Ki * 0.008
        self.kcontrol[2] = throttle.Kd * 0.3

    def pid(self):

        # self.setpoint_euler = self.setpoint_cmd * 0.02 - 30

        error = self.destination1[2] - self.gps_position[2]
        dt = self.sample_time
        self.dev = (self.error - self.prev_error)/dt
        self.prev_error = self.error
        self.ei = self.ei + self.error
        ip = self.kp*self.error + self.kd*self.dev + self.ki*self.ei

        self.pwm_cmd.prop1 = self.pwm_cmd.prop2 = self.pwm_cmd.prop3 = self.pwm_cmd.prop4 =
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    rate = rospy.Rate(Command().sample_time)
    while not rospy.is_shutdown():
        Command().pid()
        rate.sleep()
