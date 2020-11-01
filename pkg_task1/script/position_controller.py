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
        self.destination = [[19.0, 72.0, 3],
                            [19.0000451704, 72.0, 3],
                            [19.0000451704, 72.0, 0.31]]
        self.next_destination = 0
        # The threshold box can be calculated by using the tolerance of 0.000004517 in latitude, 0.0000047487 in longitude and 0.2m in altitude.

        # [roll, pitch, throttle]
        self.Kp = [676*100,  676*100,  1036*4]
        self.Ki = [34*0.008, 34*0.008,  20*0.25]
        self.Kd = [1419*40,  1419*40,   1875*1]

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
        self.Kp[0] = roll.Kp * 100
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 40

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 100
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def throttle_set_pid(self, throttle):
        self.Kp[2] = throttle.Kp * 4
        self.Ki[2] = throttle.Ki * 0.25
        self.Kd[2] = throttle.Kd * 1

    def check(self, operator):
        if operator > 2000:
            return 2000
        elif operator < 1000:
            return 1000
        else:
            return operator

    def destination_check(self):

        i = self.next_destination
        if i == 2:
            return

        if -0.000004517 < self.error[0] < 0.000004517:
            if -0.0000047487 < self.error[1] < 0.0000047487:
                if -0.2 < self.error[2] < 0.2:
                    self.next_destination += 1

    def pid(self):

        for i in [0, 1, 2]:
            self.error[i] = self.destination[self.next_destination][i] - \
                self.gps_position[i]
            self.change[i] = (
                self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + \
                self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]
            # print(self.Kp[i], self.Ki[i], self.Kd[i])
        print(self.error[1])

        print(self.output)

        self.setpoint_cmd.rcRoll = self.check(self.equilibrium_value + 100*self.output[0])
        # self.setpoint_cmd.rcRoll = self.equilibrium_value
        self.setpoint_cmd.rcPitch = self.check(self.equilibrium_value + 100*self.output[1])
        # self.setpoint_cmd.rcPitch = self.equilibrium_value
        self.setpoint_cmd.rcThrottle = self.check(self.output[2])
        self.setpoint_cmd.rcYaw = self.equilibrium_value

        self.roll_pub.publish(100000*self.error[0])
        self.pitch_pub.publish(self.error[1])
        self.throttle_pub.publish(self.error[2])
        print(self.setpoint_cmd.rcRoll)
        # print("")
        self.setpoint_pub.publish(self.setpoint_cmd)


if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    command = Command()
    rate = rospy.Rate(1/command.sample_time)
    while not rospy.is_shutdown():
        command.pid()
        command.destination_check()
        rate.sleep()
