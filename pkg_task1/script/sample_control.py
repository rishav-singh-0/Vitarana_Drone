#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from vitarana_drone.msg import prop_speed
a,a0,cnt=0,0,0
def frange(NavSatFix):
    global a,a0,cnt
    if cnt == 0:
        cnt = 1
        a0=NavSatFix.altitude
    a=NavSatFix.altitude
    
x=512.0
c=0
def stable():
    global x
    global c
    rospy.init_node("hover",anonymous=True)
    p_pub = rospy.Publisher('/edrone/pwm',prop_speed,queue_size=1)
    p_msg=prop_speed()
    rospy.Subscriber('/edrone/gps',NavSatFix,frange)

    while not rospy.is_shutdown():
        if(a>1):
            x-=0.05
            c=1
        while(c==1 and a<1):
            x+=0.05
            p_msg.prop1=p_msg.prop2=p_msg.prop3=p_msg.prop4=x
            p_pub.publish(p_msg)
            if(a==1):
                break

        

        p_msg.prop1=p_msg.prop2=p_msg.prop3=p_msg.prop4=x
        p_pub.publish(p_msg)
        print(a)

if __name__ == '__main__':
    try:
        stable()
    except rospy.ROSInterruptException:
        pass
