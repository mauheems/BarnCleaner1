import rospy
from std_msgs.msg import path
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from custom_msgs.msg import ObjectLocationArray
from sensor_msgs.msg import BatteryState
from action import Move

class MissionPlanner:
    def __init__(self):
        self.path: [PoseStamped] = []  # from the global path planner
        self.faeces_location: list = []
        self.robot_location = None
        self.path_to_follow = None
        self.status = None
        self.current_goal = None

        self.sub_obj_track = rospy.Subscriber('/tracker/feces_locations', ObjectLocationArray, self.obj_track_callback)
        self.sub_own_location = rospy.Subscriber('/acml_pose', PoseWithCovariance, self.own_location_callback)
        self.sub_battery = rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)

        self.pub_goal = rospy.Publisher('')

    def obj_track_callback(self, msg):
        pass

    def own_location_callback(self, data):
        pass
