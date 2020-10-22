#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from vitarana_drone.msg import prop_speed
current_altitude, cnt = 0, 0


def frange(NavSatFix):
    global current_altitude, a0, cnt
    if cnt == 0:
        cnt = 1
        a0 = NavSatFix.altitude
    current_altitude = NavSatFix.altitude


pwm = 512.0
desired_altitude = 3
# here is the catch
ep = dev = ei = error = input_ctrl_signal = 0
kp = 100
kd = 0.0
ki = 0.0


def stable():
    global pwm
    global desired_altitude, ep, dev, ei, error, output, kp, kd, ki, t0, t1
    rospy.init_node("hover", anonymous=True)
    p_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
    p_msg = prop_speed()
    t0 = rospy.Time.now().to_sec()
    rospy.Subscriber('/edrone/gps', NavSatFix, frange)
    ep = 0
    ei = 0
    while(True):
        error = desired_altitude - current_altitude
        output = kp*error
        ep = error

        if(output < 0.0):
            pwm = 500+(500/65.91)*output
        else:
            pwm = 500+((50/65.91)*output)

        p_msg.prop1 = p_msg.prop2 = p_msg.prop3 = p_msg.prop4 = pwm
        p_pub.publish(p_msg)


if __name__ == '__main__':
    try:
        stable()
    except rospy.ROSInterruptException:
        pass
