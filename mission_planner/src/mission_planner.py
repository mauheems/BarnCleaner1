import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point
from custom_msgs.msg import ObjectLocationArray, ObjectLocation
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

        str_serv_path = 'global_mission_planner_service'
        # print("Waiting for " + str_serv_path)
        # rospy.wait_for_service(str_serv_path)
        # self.serv_path = rospy.ServiceProxy(str_serv_path, GlobalMissionPlannerService)
        print(str_serv_path + " loaded")

        rospy.Subscriber('/tracker/feces_locations', ObjectLocationArray, self.obj_track_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.own_location_callback)
        rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        print("Subscribers and publishers created")

    @staticmethod
    def pose_array_to_path(pose: PoseArray):
        n = len(pose.poses)
        pose_numpy = np.empty((n, 4), dtype=np.float64)

        for i in range(n):
            pose_numpy[i, 0] = pose.poses[i].position.x
            pose_numpy[i, 1] = pose.poses[i].position.y
            pose_numpy[i, 2] = pose.poses[i].position.z
        return pose_numpy
    
    @staticmethod
    def point_to_array(point: Point):
        arr = np.array([point.x, point.y, point.z])
        return arr
    
    def distance_calc(point):
        return

    def obj_track_callback(self, data: ObjectLocationArray):
        d = ObjectLocation()
        d_arr = self.point_to_array(d.abs_location)
        diff = self.path_numpy - d_arr
        dist = np.linalg.norm(diff, ord=2, axis=1)
        min_ind = np.argmin(dist)
        if min_ind == 0:
            insert_ind = 1
        elif min_ind == len(diff) - 1:
            insert_ind = min_ind - 1
        else:
            next_ind = min_ind + 1 if dist[min_ind + 1] < dist[min_ind - 1] else min_ind - 1
        self.path_numpy = None
        return

    def own_location_callback(self, data: PoseWithCovarianceStamped):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        print(position, orientation)
        return

    def battery_callback(self, data: BatteryState):
        self.batter_percentage = data.percentage
        # print(self.batter_percentage)
        if data.percentage < 0.2:
            # Set next goal position as Home
            pass
        pass
