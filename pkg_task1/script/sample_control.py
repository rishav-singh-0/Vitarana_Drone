#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from vitarana_drone.msg import prop_speed
from pid_tune.msg import PidTune
current_altitude, cnt = 0, 0

kp = ki = kd = 0


def frange(NavSatFix):
    global current_altitude, a0, cnt
    if cnt == 0:
        cnt = 1
        a0 = NavSatFix.altitude
    current_altitude = NavSatFix.altitude


def pid(PidTune):
    global kp, kd, ki
    kp = PidTune.Kp/5
    kd = PidTune.Kd
    ki = PidTune.Ki/10
    # print(kp, ki, kd)
    # here i have taken the kp=208,kd=75,ki=0


pwm = 512.0
desired_altitude = 3
# here is the catch
ep = dev = ei = error = input_ctrl_signal = 0


def stable():
    global pwm
    global desired_altitude, ep, dev, ei, error, output, kp, ki, kd
    rospy.init_node("hover", anonymous=True)
    p_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
    p_msg = prop_speed()
    rospy.Subscriber('/edrone/gps', NavSatFix, frange)
    ep = 0
    ei = 0
    while(True):
        rospy.Subscriber("/pid_tuning", PidTune, pid)
        error = desired_altitude - (current_altitude-0.3099987281832533)
        dt = 0.01
        dev = (error-ep)/dt
        ep = error
        ei = ei + error*dt
        output = kp*error + ki*ei + kd*dev + 500.0
        if(output < 0):
            output = 0

        p_msg.prop1 = p_msg.prop2 = p_msg.prop3 = p_msg.prop4 = output
        print(output, current_altitude)
        p_pub.publish(p_msg)
        rospy.Rate(100).sleep()


if __name__ == '__main__':
    try:
        stable()
    except rospy.ROSInterruptException:
        pass
