#!/usr/bin/env python
import rospy
from vitarana_drone.msg import prop_speed


def myhook():
    print "shutdown time!"
    pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
    pwm_holder = prop_speed()
    pwm_holder.prop1 = pwm_holder.prop2 = pwm_holder.prop3 = pwm_holder.prop4 = 0
    pwm_pub.publish(pwm_holder)


rospy.init_node('shutdown_hook')
rospy.on_shutdown(myhook)
