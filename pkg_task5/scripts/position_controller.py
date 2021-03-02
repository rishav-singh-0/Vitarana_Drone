#!/usr/bin/env python

'''
This python file runs a ROS-node of name 'position_controller' which controls the required roll, pitch, yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /drone_command          /edrone/gps
                                /checkpoint
'''

# Team ID:          VD_983
# Theme:            Vitarana_Drone
# Author List:      Rishav Singh, Kashyap Joshi
# Filename:         position_controller.py
# Functions:        pid, gps_callback, checkpoint_callabck, check_output
# Global variables: None

import rospy
from vitarana_drone.msg import edrone_cmd
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

class Command():
    '''
    Purpose:
    ---
    Navigating the Drone through commands
    Take data from path_planner and process it using pid 
    for requirements of roll, pitch, yaw and throttle 

    Input Arguments:
    ---
    None
    '''
    # constructor

    def __init__(self):
        rospy.init_node('position_controller')

        # initialising gps position
        # [latitude,longitude,altitude]
        self.gps_position = [0, 0, 0]

        # initialising desired position
        # [latitude,longitude,altitude]
        self.destination = [0, 0, 0]

        # [roll, pitch, throttle]
        self.Kp = [1507*10000,  1507*10000,  290*1]
        self.Ki = [0, 0,  0*0.25]
        self.Kd = [1040*10000*5,  1040*10000*5,   284*1]
        # necessary variables for calculation of desired position for roll,pitch and throttle
        # [roll, pitch, throttle]sss
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

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/checkpoint', NavSatFix, self.checkpoint_callback)

    def gps_callback(self, msg):
        '''
        Purpose:
        ---
        Stores the data given by the gps sensor inside gps_position

        Input Arguments:
        ---
        msg :  [ NavSatFix ]
            raw data from gps sensor

        Returns:
        ---
        None

        Example call:
        ---
        gps_callback(<gps_data>)
        '''

        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def checkpoint_callback(self, msg):
        ''''''
        '''
        Purpose:
        ---
        Taking checkpoints from path_planner.py and store in destination

        Input Arguments:
        ---
        msg :  [ NavSatFix ]
            next destination to be travelled 

        Returns:
        ---
        None

        Example call:
        ---
        checkpoint_callback(<NavSatFix_data>)
        '''
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def check(self, operator):
        '''
        Purpose:
        ---
        Verifying if the value is within range if not making it and transforming it for desired range

        Input Arguments:
        ---
        operator :  [ float32 ]
            output of pid control system

        Returns:
        ---
        None

        Example call:
        ---
        check_output(1500)
        '''
        operator = self.equilibrium_value + operator
        if operator > 2000:
            return 2000
        elif operator < 1000:
            return 1000
        else:
            return operator

    def pid(self):
        '''
        Purpose:
        ---
        Calculates required roll, pitch, yaw and throttle for reaching checkpoint provided by path_planner

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        pid()
        '''

        for i in range(3):
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
        self.setpoint_pub.publish(self.setpoint_cmd)

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the Edrone class to control eDrones movement
if __name__ == '__main__':
    # specify rate in Hz based upon your desired PID sampling time
    command = Command()
    rate = rospy.Rate(1/command.sample_time)  # defining rate
    while not rospy.is_shutdown():
        command.pid()
        rate.sleep()  # frequency of 100 Hz
