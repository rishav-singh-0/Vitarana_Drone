#!/usr/bin/env python
import rospy
from pid_tune.msg import PidTune
kp = kd = ki = 0


def get_const(PidTune):
    global kp, kd, ki
    kp = PidTune.Kp
    kd = PidTune.Kd
    ki = PidTune.Ki
    print(kp, kd, ki)


rospy.init_node("pid_constants", anonymous=True)
rospy.Subscriber('/pid_tuning', PidTune, get_const)

while True:
    print(kp, ki, kd)
