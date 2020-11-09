#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries
import tf
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from vitarana_drone.msg import *
import math


class Edrone():
    """docstring for Edrone"""

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
        self.sample_time = 0.01  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        self.zero_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd,
                         self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        # ------------------------------------------------------------------------------------------------------------

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msgs/Imu

    def check(self, operator):
        if operator > self.max_values:
            operator = self.max_values
        if operator < self.min_values:
            operator = self.min_values

    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.rcThrottle = msg.rcThrottle

        # ---------------------------------------------------------------------------------------------------------------

    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp*0.01
        self.Ki[0] = roll.Ki*0.008
        self.Kd[0] = roll.Kd*0.01

    # This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.01
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.01

    # This function gets executed each time when /tune_pid publishes /pid_tuning_yaw
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.06
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 0.01

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        # - 1. Convert the quaternion format of orientation to euler angles
        # - 2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        # - 3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
        # - 4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        # * 5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        # - 6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        # - 7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        # - 8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        # - 8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        # - 9. Add error_sum to use for integral component

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll, pitch and yaw axis
        def degree_convert(operator): return operator * 102.4

        for i in range(3):
            self.setpoint_euler[i] = self.setpoint_cmd[i] * 0.02 - 30
        # self.setpoint_euler[3] = self.setpoint_cmd[3] * 0.02 - 30

        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
        # Because of physical limitations prop speed will never reach its max speed
        #

        self.throttle_cmd = self.rcThrottle - 1000 + 12

        for i in range(3):
            self.error[i] = self.setpoint_euler[i] - \
                math.degrees(self.drone_orientation_euler[i])
            self.change[i] = (self.error[i]-self.prev_error[i]
                              ) / self.sample_time
            self.prev_error[i] = self.error[i]
            self.sum[i] = self.sum[i] + self.error[i] * self.sample_time
            self.output[i] = self.Kp[i] * self.error[i] + \
                self.Kd[i]*self.change[i] + self.Ki[i]*self.sum[i]
            # print(i, math.degrees(self.drone_orientation_euler[i]))

        # converting range 1000 t0 2000 to 0 to 1024
        self.roll_cmd = degree_convert(self.output[0])
        self.pitch_cmd = degree_convert(self.output[1])
        self.yaw_cmd = degree_convert(self.output[2])

        self.pwm_cmd.prop1 = self.throttle_cmd - \
            self.roll_cmd + self.pitch_cmd - self.yaw_cmd
        self.pwm_cmd.prop2 = self.throttle_cmd -  \
            self.roll_cmd - self.pitch_cmd + self.yaw_cmd
        self.pwm_cmd.prop3 = self.throttle_cmd + \
            self.roll_cmd - self.pitch_cmd - self.yaw_cmd
        self.pwm_cmd.prop4 = self.throttle_cmd + \
            self.roll_cmd + self.pitch_cmd + self.yaw_cmd

        self.check(self.pwm_cmd.prop1)
        self.check(self.pwm_cmd.prop2)
        self.check(self.pwm_cmd.prop3)
        self.check(self.pwm_cmd.prop4)

        self.roll_pub.publish(self.error[0])
        self.pitch_pub.publish(self.error[1])
        self.yaw_pub.publish(self.error[2])
        self.zero_pub.publish(0)
        # print(self.pwm_cmd)
        # print("")
        # print(math.degrees(self.drone_orientation_euler[1]))
        # print(math.degrees(self.drone_orientation_euler[0]), "\n", math.degrees(
        #     self.drone_orientation_euler[1]), "\n", math.degrees(self.drone_orientation_euler[2]), "\n\n")
        self.pwm_pub.publish(self.pwm_cmd)
        self.roll_pub.publish(self.error[i])
        # ------------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':

    e_drone = Edrone()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(1/e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
