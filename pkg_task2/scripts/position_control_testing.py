#!/usr/bin/env python
# importing required libraries
import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# class for position_controller.py
global destination
destination=[]

class Command():
    '''Navigating the Drone through commands'''
    # constructor

    def __init__(self):
        rospy.init_node('position_controller')

        # initialising gps position
        # [latitude,longitude,altitude]
        self.gps_position = [0, 0, 0]
        self.attech_situation = "False"

        # initialising desired position
        # [latitude,longitude,altitude]
        # self.destination = [[19.0009248718, 71.9998318945, 25.1599967919],
        #                     [19.0007046575,  71.9998955286, 25.1599967919],
        #                     [19.0007046575, 71.9998955286, 22.1599967919],[19.0007046575,  71.9998955286, 25.1599967919]]
        self.next_destination = 0
        self.destination=[0,0,0]
        self.box_flag = 0
        self.index=0#it will handle all the positions status
        self.index_endpoint=0#it will limits the index to reach at any point
        self.feedback_to_path_planner=0#when destination is reached it will give feedback to path_planner so it can send new points
        self.subpoint=0#this flag will give index to path_planner
        self.cnt=0

        self.check_flag=0
        # necessary variables for calculation of desired position for roll,pitch and throttle
        # [roll, pitch, throttle]
        # self.Kp = [2744*10000,  2744*10000,  375]
        # self.Ki = [8.0*0.008, 8*0.008,  3*0.25]
        # self.Kd = [1381*10000*5, 1381*10000*5,  354]

        self.Kp = [1007*10000,  1007*10000,  375]
        self.Ki = [7*0.008, 7*0.008,  3*0.25]
        self.Kd = [592*10000*5, 592*10000*5,  354]

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
        self.setpoint_pub = rospy.Publisher(
            '/drone_command', edrone_cmd, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.op_pub = rospy.Publisher('op_flag', Float32, queue_size=1)
        #self.check_point_flag = rospy.Publisher('check_point_flag', Float32, queue_size=1)#it will simply give a signal that where to take the QR code value
        self.subpoint_flag = rospy.Publisher('subpoint_flag', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher(
            '/throttle_error', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.call_back)
        rospy.Subscriber('/checkpoint', NavSatFix, self.checkpoint_callback)
        # rospy.Subscriber('/pid_tuning_altitude',
        #                  PidTune, self.throttle_set_pid)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
    
    def checkpoint_callback(self,msg):
        container=[msg.latitude, msg.longitude, msg.altitude]
        if(self.cnt==0 and self.destination!=container):
            self.destination=container
            print(self.destination)
            self.cnt=1

        #print(self.destination)

    def gps_callback(self, msg):
        self.gps_position = [msg.latitude, msg.longitude, msg.altitude]

    def call_back(self, state):
        self.attech_situation = state.data

    # def roll_set_pid(self, roll):
    #     self.Kp[0] = roll.Kp * 10000
    #     self.Ki[0] = roll.Ki * 0.008
    #     self.Kd[0] = roll.Kd * 10000*5

    # def pitch_set_pid(self, pitch):
    #     self.Kp[1] = pitch.Kp * 10000
    #     self.Ki[1] = pitch.Ki * 0.008
    #     self.Kd[1] = pitch.Kd * 10000*5

    # def throttle_set_pid(self, throttle):
    #     self.Kp[2] = throttle.Kp * 1
    #     self.Ki[2] = throttle.Ki * 0.25
    #     self.Kd[2] = throttle.Kd * 1

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
        
        self.index = self.next_destination
        # if self.index == self.index_endpoint:
        #     return

        if -0.000004517 <= self.error[0] <= 0.000004517:
            if -0.0000047487 <= self.error[1] <= 0.0000047487:
                 if -0.2 <= self.error[2] <= 0.2:
                     self.cnt=0
                     print(self.cnt)
                        # self.next_destination+=1
                        # print()
                        # print(self.index)
                        #self.subpoint_flag.publish(self.next_destination)


                    # self.next_destination += 1
                    # print("destination reached")
    def pid(self):
        '''Function for implimenting the pid algorithm'''
        for i in range(3):
            self.error[i] = self.destination[i] - \
                self.gps_position[i]
            self.change[i] = (
                self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + \
                self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]
        # print(self.gps_position[2])
        # print(self.error[2])
        if(round(self.gps_position[2], 1) == 25.2 and self.index == 5):
            self.box_flag = 1
            print("box will detech by itself")
            

        # figure out the values  for roll,pitch and throttle
        self.setpoint_cmd.rcRoll = self.check(self.output[0])
        self.setpoint_cmd.rcPitch = self.check(self.output[1])
        self.setpoint_cmd.rcThrottle = self.check(self.output[2])
        self.setpoint_cmd.rcYaw = self.equilibrium_value

        # publishing all the values to attitude_controller and for plotting purpose
        self.roll_pub.publish(self.error[0])
        self.op_pub.publish(self.box_flag)
        self.pitch_pub.publish(self.error[1])
        self.throttle_pub.publish(self.error[2])
        self.setpoint_pub.publish(self.setpoint_cmd)
        
        

        # print(self.j)
if __name__ == '__main__':

    # specify rate in Hz based upon your desired PID sampling time
    command = Command()
    rate = rospy.Rate(1/command.sample_time)  # defining rate
    while not rospy.is_shutdown():
            command.pid()
            command.destination_check()
            rate.sleep()  # frequency of 100 Hz
