import rospy
from sensor_msgs.msg import NavSatFix, LaserScan


class PathPlanner():

    def __init__(self):
        rospy.init_node('/path_planner')

        # Destination to be reached
        # [latitude, longitude, altitude]
        self.destination = [0, 0, 0]

        # Present Location of the DroneNote
        self.current_location = [0, 0, 0]

        # The checkpoint node to be reached for reaching final destination
        self.checkpoint = NavSatFix()

        # 
        self.obs_range_top = LaserScan()
        self.obs_range_bottom = LaserScan()

        self.yaw_error = 0


        # Publisher
        self.pub_checkpoint = rospy.Publisher('/checkpoint', NavSatFix, queue_size=1)

        # Subscriber
        rospy.Subscriber('/final_setpoint', NavSatFix, self.final_setpoint_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_bottom_callback)

    def final_setpoint_callback(self, msg):
        self.destination = [msg.latitude, msg.longitude, msg.altitude]

    def gps_callback(self, msg):
        self.current_location = [msg.latitude, msg.longitude, msg.altitude]

    def range_finder_top(self, msg):
        self.obs_range_top = msg.ranges

    def range_finder_bottom(self, msg):
        self.obs_range_bottom = msg.ranges
    
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)


    def scan(self):
        '''For Processing the obtained sensor data'''
        # yaw to 10 degree
        
        return

    def planner(self):

        ##############################################################
        # Main Algorithm
        ##############################################################

        self.checkpoint.latitude = 0
        self.checkpoint.longitude = 0
        self.checkpoint.altitude = 0

        self.pub_checkpoint.publish(self.checkpoint)
