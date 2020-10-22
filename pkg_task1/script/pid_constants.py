#!/usr/bin/env python
import rospy
from pid_tune.msg import PidTune
kp = kd = ki = 0


def get_const(PidTune):
    global kp, kd, ki
    kp = PidTune.Kp
    kd = PidTune.Kd
    ki = PidTune.Ki

    pass


rospy.Subscriber('turtle1/pose', PidTune, get_cost)
