import rospy
from sensor_msgs.msg import NavSatFix, LaserScan


class PathPlanner():

    def __init__(self):
        rospy.init_node('/path_planner')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]

        # Present Location of the Drone
        self.current_location = [0, 0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()

        # 
        self.obs_range_top = LaserScan()
        self.obs_range_bottom = LaserScan()

        # Publisher
        self.pub_checkpoint = rospy.Publisher(
            '/checkpoint', NavSatFix, queue_size=1)

        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, final_setpoint_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, range_finder_bottom_callback)

    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top(self, msg):
        self.obs_range_top = msg.ranges

    def range_finder_bottom(self, msg):
        self.obs_range_bottom = msg.ranges

    def data_proccessing(self):
        '''For Processing the obtained sensor data'''
        return

    def planner(self):

        ##############################################################
        # Main Algorithm
        ##############################################################

        self.checkpoint.latitude = 0
        self.checkpoint.longitude = 0
        self.checkpoint.altitude = 0

        self.pub_checkpoint.publish(self.checkpoint)
