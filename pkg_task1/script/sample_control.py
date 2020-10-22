#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from vitarana_drone.msg import prop_speed
current_altitude,a0,cnt=0,0,0
def frange(NavSatFix):
    global current_altitude,a0,cnt
    if cnt == 0:
        cnt = 1
        a0=NavSatFix.altitude
    current_altitude=NavSatFix.altitude
    
pwm=512.0
desired_altitude=2
#here is the catch
ep=ed=ei=ec=input_ctrl_signal=t0=t1=0
kp=0.01
kd=0.2
ki=0.5
def stable():
    global pwm
    global desired_altitude,ep,ed,ei,ec,input_ctrl_signal,kp,kd,ki,t0,t1
    rospy.init_node("hover",anonymous=True)
    p_pub = rospy.Publisher('/edrone/pwm',prop_speed,queue_size=1)
    p_msg=prop_speed()
    t0 = rospy.Time.now().to_sec()
    rospy.Subscriber('/edrone/gps',NavSatFix,frange)
    ep=current_altitude
    ei=0
    while(True):
        ec=desired_altitude-ep
        t1 = rospy.Time.now().to_sec()
        ed=(ep-ec)
        ei=ei+ec*(t1-t0)
        ep=ec
        input_ctrl_signal= kp*ec + kd*ed + ki*ei
        print(input_ctrl_signal,"and altitude is",current_altitude)
        p_msg.prop1=p_msg.prop2=p_msg.prop3=p_msg.prop4=pwm
        p_pub.publish(p_msg)
        #print()

if __name__ == '__main__':
    try:
        stable()
    except rospy.ROSInterruptException:
        pass
