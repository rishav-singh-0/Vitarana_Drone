#!/usr/bin/env python

# importing required libraries

import rospy
from vitarana_drone.msg import edrone_cmd
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
        self.destination = [0, 0, 0]
        # The threshold box can be calculated by using the tolerance of 0.000004517 in latitude, 0.0000047487 in longitude and 0.2m in altitude.

        # [roll, pitch, throttle]
        self.Kp = [1507*10000,  1507*10000,  290*1]
        self.Ki = [0, 0,  0*0.25]
        self.Kd = [655*10000*5,  655*10000*5,   284*1]
        # necessary variables for calculation of desired position for roll,pitch and throttle
        # [roll, pitch, throttle]
        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.change = [0, 0, 0]
        self.sum = [0, 0, 0]
        self.output = [0, 0, 0]

        self.sample_time = 0.1  # sample time

        # equilibrium point for all the angle(roll,pitch,yaw) and throttle
        self.equilibrium_value = 1500
        
        # object for publishing the rc messages
        self.setpoint_cmd = edrone_cmd()

        # Publishing /pitch_error, /roll_error, /throttle_error
        self.setpoint_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        # self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        # self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_error', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/destination', NavSatFix, self.destination_callback)

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def destination_callback(self, msg):
        self.destination[0] = msg.latitude
        self.destination[1] = msg.longitude
        self.destination[2] = msg.altitude


    # this function will convert all rc messages in the range of 1000 to 2000
    def check(self, operator):
        ''' Vreifying if the value is within range if not making it and transforming it for desired range'''
        operator = self.equilibrium_value + operator
        if operator > 2000:
            return 2000
        elif operator < 1000:
            return 1000
        else:
            return operator

    def pid(self):

        for i in range(3):
            self.error[i] = self.destination[i] - self.gps_position[i]
            self.change[i] = (self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]

        # figure out the values  for roll,pitch and throttle
        self.setpoint_cmd.rcRoll = self.check(self.equilibrium_value + self.output[0])
        self.setpoint_cmd.rcPitch = self.check(self.equilibrium_value + self.output[1])
        self.setpoint_cmd.rcThrottle = self.check(self.equilibrium_value + self.output[2])
        self.setpoint_cmd.rcYaw = self.equilibrium_value

        # publishing all the values to attitude_controller and for plotting purpose
        # self.roll_pub.publish(self.error[0])
        # self.pitch_pub.publish(self.error[1])
        self.throttle_pub.publish(self.error[2])
        self.setpoint_pub.publish(self.setpoint_cmd)


if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    command = Command()
    rate = rospy.Rate(1/command.sample_time)  # defining rate
    while not rospy.is_shutdown():
        command.pid()
        rate.sleep()  # frequency of 100 Hz
