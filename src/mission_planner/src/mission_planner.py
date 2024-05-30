import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from custom_msgs.msg import ObjectLocationArray
from sensor_msgs.msg import BatteryState
import numpy as np

class MissionPlanner:
    def __init__(self):
        print("MissionPlanner node initialized")
        self.robot_id : int = None
        self.path: list = []  # from the global path planner
        self.path_numpy: np.ndarray = None
        self.faeces_location: list = []
        self.robot_location = None
        self.path_numpy_to_follow = None  # stores the current path to follow
        self.status = None
        self.current_goal = None

        # str_serv_path = 'global_mission_planner_service'
        # rospy.wait_for_service(str_serv_path)
        # self.serv_path = rospy.ServiceProxy(str_serv_path, GLOB)

        rospy.Subscriber('/tracker/feces_locations', ObjectLocationArray, self.obj_track_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.own_location_callback)
        rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def pose_array_to_path(self, pose: PoseArray):
        n = len(pose.poses)
        pose_numpy = np.empty((n, 3), dtype=np.float64)

        for i in range(n):
            pose_numpy[i, 0] = pose.poses[i].position.x
            pose_numpy[i, 1] = pose.poses[i].position.y
            pose_numpy[i, 2] = pose.poses[i].position.z
        
        self.path_numpy = pose_numpy
        self.path = pose.poses
        return
    
    def distance_calc(point):
        return

    def obj_track_callback(self, data):
        
        return

    def own_location_callback(self, data: PoseWithCovarianceStamped):
        position = data.pose.pose.position
        print(position)
        return

    def battery_callback(self, data: BatteryState):
        # print(data.percentage)
        pass
