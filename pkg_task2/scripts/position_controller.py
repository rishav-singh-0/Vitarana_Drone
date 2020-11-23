#!/usr/bin/env python

# importing required libraries

import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import NavSatFix

# class for position_controller.py


class Command():
    '''Navigating the Drone through commands'''
    # constructor

    def __init__(self):
        rospy.init_node('position_controller')

        # initialising gps position
        # [latitude,longitude,altitude]
        self.gps_position = [0, 0, 0]

        # initialising desired position
        # [latitude,longitude,altitude]
        self.destination = [0, 0, 0]            # destination ehere drone is currrently heded
        self.final_destination = [0, 0, 0]      # container for final destination 
        
        # this will be true if drone has reached its currently provided setpoint
        self.take_destination = True            

        # necessary variables for calculation of desired position for roll,pitch and throttle
        # [roll, pitch, throttle]
        self.Kp = [1007*10000,  1097*10000,  375]
        self.Ki = [7*0.008, 7*0.008,  3*0.25]
        self.Kd = [592*50000, 590*50000,  354]

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
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_error', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/checkpoint', NavSatFix, self.checkpoint_callback)
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_destination_callback)

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def checkpoint_callback(self, msg):
        '''Taking checkpoints from path_planner.py'''

        container = [msg.latitude, msg.longitude, msg.altitude]

        if self.take_destination:
            # checking if the drone is hovering over final destination
            if(-0.00001017 < (self.final_destination[0]-self.gps_position[0]) < 0.00001017):
                self.destination = self.final_destination
            else:
              self.destination = container
            self.take_destination = False

    def final_destination_callback(self, msg):
        self.final_destination = [msg.latitude, msg.longitude, msg.altitude]


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

    def destination_check(self):
        ''' function will hendle all desired positions '''
        if -0.000010517 <= self.error[0] <= 0.000010517:
            if -0.0000127487 <= self.error[1] <= 0.0000127487:
                if -0.1 <= self.error[2] <= 0.1:
                    self.take_destination = True
                    # print("destination reached")


    def pid(self):
        '''Function for implimenting the pid algorithm'''

        for i in range(3):
            # rospy.loginfo(self.destination)
            self.error[i] = self.destination[i] - self.gps_position[i]
            self.change[i] = (self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]


        # figure out the values  for roll,pitch and throttle
        self.setpoint_cmd.rcRoll = self.check(self.output[0])
        self.setpoint_cmd.rcPitch = self.check(self.output[1])
        self.setpoint_cmd.rcThrottle = self.check(self.output[2])
        self.setpoint_cmd.rcYaw = self.equilibrium_value

        # publishing all the values to attitude_controller and for plotting purpose
        self.roll_pub.publish(self.error[0])
        self.pitch_pub.publish(self.error[1])
        self.throttle_pub.publish(self.error[2])
        self.setpoint_pub.publish(self.setpoint_cmd)


if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    command = Command()
    rate = rospy.Rate(1/command.sample_time)  # defining rate
    while not rospy.is_shutdown():
        command.pid()
        command.destination_check()
        rate.sleep()  # frequency of 100 Hz
