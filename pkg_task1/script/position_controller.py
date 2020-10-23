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

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.prev_error = 0
        # Taking max value of prop speed 1000 not 1024 cause in real world it is impossible to operate motors on its maximum speed
        self.max_values = [1000, 1000, 1000, 1000]
        self.max_values = [0, 0, 0, 0]

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
        self.kcontrol[0] = pitch.Kp * 0.06
        self.kcontrol[1] = pitch.Ki * 0.008
        self.kcontrol[2] = pitch.Kd * 0.3

    def pid(self):

        # self.setpoint_euler = self.setpoint_cmd * 0.02 - 30
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    rate = rospy.Rate(e_drone.sample_time)
    while not rospy.is_shutdown():
        Command().pid()
        rate.sleep()
