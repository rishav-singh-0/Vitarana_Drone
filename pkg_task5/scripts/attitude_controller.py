#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /edrone/drone_command
        /pitch_error            /edrone/imu/data
        /yaw_error
        /edrone/pwm

'''

# Team ID:          VD_983
# Theme:            Vitarana_Drone
# Author List:      Rishav Singh, Kashyap Joshi
# Filename:         attitude_controller.py
# Functions:        pid, shutdown_hook, imu_callback, drone_command_callback, check_output
# Global variables: None

# Importing the required libraries
import tf
import time
import rospy
from math import degrees
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from pid_tune.msg import PidTune
from vitarana_drone.msg import prop_speed, edrone_cmd
from std_srvs.srv import Empty


class Edrone():
    '''
    Purpose:
    ---
    Take data from position_controller and process it using pid 
    for requirements of propeller speed

    Input Arguments:
    ---
    None
    '''

    def __init__(self):
        # initializing ros node with name attitude_controller
        rospy.init_node('attitude_controller')

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [26*0.01, 26*0.01, 93*0.06]
        self.Ki = [0, 0, 1*0.008]
        self.Kd = [10*0.01, 10*0.01, 2*0.01]
        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.change = [0, 0, 0]
        self.sum = [0, 0, 0]
        self.output = [0, 0, 0]
        # Taking max value of prop speed 1024
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]

        # initialising rcThrottle which is to be subscribed
        self.rcThrottle = 0

        self.roll_cmd = 0
        self.pitch_cmd = 0
        self.yaw_cmd = 0

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.010  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        # self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        # self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        # self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

        #ShutdownHook
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)

        # ------------------------------------------------------------------------------------------------------------

    def check_output(self, operator):
        '''
        Purpose:
        ---
        Vreifying if prop speed is within range if not making it

        Input Arguments:
        ---
        operator :  [ float32 ]
            output of pid control system

        Returns:
        ---
        None

        Example call:
        ---
        check_output(1024)
        '''

        if operator > self.max_values:
            operator = self.max_values
        elif operator < self.min_values:
            operator = self.min_values

    def imu_callback(self, msg):
        '''
        Purpose:
        ---
        Stores the data given by the imu sensor

        Input Arguments:
        ---
        msg :  [ Imu ]
            Raw data from imu sensor

        Returns:
        ---
        None

        Example call:
        ---
        imu_callback(<Imu_message>)
        '''
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        '''
        Purpose:
        ---
        Stores the data given by the position_controller 

        Input Arguments:
        ---
        msg :  [ edrone_cmd ]
            Raw data from position_controller about required set values

        Returns:
        ---
        None

        Example call:
        ---
        drone_command_callback(<edrone_cmd>)
        '''

        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.rcThrottle = msg.rcThrottle

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        '''
        Purpose:
        ---
        Controls propeller speed of eDrone from the provided data of position_controller

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

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll, pitch and yaw axis
        def degree_convert(operator): return operator * 102.4

        for i in range(3):
            self.setpoint_euler[i] = self.setpoint_cmd[i] * 0.024 - 36

        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
        # Because of physical limitations prop speed will never reach its max speed
        self.throttle_cmd = (self.rcThrottle - 1000) * 1.024

        for i in range(3):
            self.error[i] = self.setpoint_euler[i] - degrees(self.drone_orientation_euler[i])
            self.change[i] = (self.error[i] - self.prev_error[i]) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]

        # Converting range 1000 to 2000 to degrees
        self.roll_cmd = degree_convert(self.output[0])
        self.pitch_cmd = degree_convert(self.output[1])
        self.yaw_cmd = degree_convert(self.output[2])

        self.pwm_cmd.prop1 = self.throttle_cmd - self.roll_cmd + self.pitch_cmd - self.yaw_cmd
        self.pwm_cmd.prop2 = self.throttle_cmd - self.roll_cmd - self.pitch_cmd + self.yaw_cmd
        self.pwm_cmd.prop3 = self.throttle_cmd + self.roll_cmd - self.pitch_cmd - self.yaw_cmd
        self.pwm_cmd.prop4 = self.throttle_cmd + self.roll_cmd + self.pitch_cmd + self.yaw_cmd

        # Vreifying if prop speed is within range if not making it
        self.check_output(self.pwm_cmd.prop1)
        self.check_output(self.pwm_cmd.prop2)
        self.check_output(self.pwm_cmd.prop3)
        self.check_output(self.pwm_cmd.prop4)

        # Publishing error messages
        # self.roll_pub.publish(self.error[0])
        # self.pitch_pub.publish(self.error[1])
        # self.yaw_pub.publish(self.error[2])

        # Publishing Prop Speeds
        self.pwm_pub.publish(self.pwm_cmd)
        # ------------------------------------------------------------------------------------------------------------------------

    def shutdown_hook(self):
        '''
        Purpose:
        ---
        Shutdown hooks works when the script is called to be killed
        It restores the position of eDrone and changes the propeller speed to 0

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        shutdown_hook()
        '''
        
        self.pwm_cmd.prop1 = self.pwm_cmd.prop2 = self.pwm_cmd.prop3 = self.pwm_cmd.prop4 = 0
        self.pwm_pub.publish(self.pwm_cmd)
        self.reset_world()

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the Edrone class to control eDrones movement
if __name__ == '__main__':

    e_drone = Edrone()
    # specify rate in Hz based upon your desired PID sampling time
    # i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(1/e_drone.sample_time)
    while not rospy.is_shutdown():
        # running with a desired frequency
        e_drone.pid()
        rospy.on_shutdown(e_drone.shutdown_hook)
        r.sleep()
