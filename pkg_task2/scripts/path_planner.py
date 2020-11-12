import rospy
from sensor_msgs.msg import NavSatFix


class PathPlanner():

    def __init__(self):
        rospy.init_node('/path_planner')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]

        # Present Location of the Drone
        self.current_location = [0, 0, 0]

        self.checkpoint = NavSatFix()

        # Publisher
        self.pub_checkpoint = rospy.Publisher(
            '/checkpoint', NavSatFix, queue_size=1)

        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, final_setpoint_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, gps_callback)

    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def planner(self):

        ##############################################################
        # Main Algorithm
        ##############################################################

        self.checkpoint.latitude = 0
        self.checkpoint.longitude = 0
        self.checkpoint.altitude = 0

        self.pub_checkpoint.publish(self.checkpoint)
