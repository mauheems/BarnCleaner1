import rospy
from std_msgs.msg import path
from geometry_msgs.msg import PoseStamped


class MissionPlanner:
    def __init__(self):
        self.path: [PoseStamped] = []  # from the global path planner
        self.faeces_location: list = []
        self.robot_location = None
        self.path_to_follow = None
        self.status = None
        self.current_goal = None

        self.sub_obj_track = rospy.Subscriber('')
