#!/usr/bin/env python

# importing required libraries

import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
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
        self.destination1 = [19.0, 72.0, 3]
        self.destination2 = [19.0000451704, 72.0, 3]
        self.destination3 = [19.0000451704, 72.0, 0.31]
        # The threshold box can be calculated by using the tolerance of 0.000004517 in latitude, 0.0000047487 in longitude and 0.2m in altitude.

        # [roll, pitch, throttle]
        self.Kp = [0, 0, 852*4]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 3775*0.5]

        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.change = [0, 0, 0]
        self.sum = [0, 0, 0]
        self.output = [0, 0, 0]

        self.sample_time = 0.010
        self.equilibrium_value = 1500

        self.setpoint_cmd = edrone_cmd()

        # Publishing /pitch_error, /roll_error, /throttle_error
        self.setpoint_pub = rospy.Publisher(
            '/drone_command', edrone_cmd, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher(
            '/throttle_error', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/pid_tuning_altitude',
        #                  PidTune, self.throttle_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp*10
        self.Ki[0] = roll.Ki
        self.Kd[0] = roll.Kd

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 10000
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def throttle_set_pid(self, throttle):
        self.Kp[2] = throttle.Kp * 4
        self.Ki[2] = throttle.Ki * 0.25
        self.Kd[2] = throttle.Kd * 0.5

    def check(self, operator):
        if operator > 2000:
            return 2000
        elif operator < 1000:
            return 1000
        else:
            return operator

    def pid(self):

        for i in range(2, 3):
            self.error[i] = self.destination1[i] - self.gps_position[i]
            self.change[i] = (
                self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + \
                self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]
            # print(self.Kp[i], self.Ki[i], self.Kd[i])
        if(round(self.gps_position[2], 1) == 2.6):
            for i in range(1):
                self.error[i] = 10000 * \
                    (self.destination2[i]-self.gps_position[i])
                self.change[i] = (
                    self.error[i] - self.prev_error[i]) / self.sample_time
                self.prev_error[i] = self.error[i]
                self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
                self.output[i] = self.Kp[i] * self.error[i] + \
                    self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]
                self.setpoint_cmd.rcRoll = self.check(self.output[0])
                print(self.output[0])
                print(self.error[i])
            #print(self.Kp[i], self.Ki[i], self.Kd[i])
        else:
            self.setpoint_cmd.rcRoll = self.equilibrium_value

        # print(self.error[2])
        # print()
        # print(self.gps_position[2])

        #self.setpoint_cmd.rcRoll = self.check(self.output[1])
        #self.setpoint_cmd.rcRoll = self.equilibrium_value
        self.setpoint_cmd.rcPitch = self.equilibrium_value
        #self.setpoint_cmd.rcPitch = self.check(self.output[0])
        self.setpoint_cmd.rcThrottle = self.check(self.output[2])
        self.setpoint_cmd.rcYaw = self.equilibrium_value

        self.roll_pub.publish((self.error[0]))
        self.pitch_pub.publish((self.error[1]))
        self.throttle_pub.publish(self.error[2])
        # print(self.setpoint_cmd.rcThrottle)
        # print("")
        self.setpoint_pub.publish(self.setpoint_cmd)


if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    command = Command()
    rate = rospy.Rate(1/command.sample_time)
    while not rospy.is_shutdown():
        command.pid()
        rate.sleep()
