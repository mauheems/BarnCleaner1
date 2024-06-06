import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point
from custom_msgs.msg import ObjectLocationArray, ObjectLocation
from sensor_msgs.msg import BatteryState
from mission_planner.srv import *
import numpy as np

class MissionPlanner:
    def __init__(self):
        print("MissionPlanner node initialized")
        self.robot_id : int = None
        self.path_numpy: np.ndarray = None
        self.faeces_location: list = []
        self.robot_posn = None
        self.path_numpy_to_follow = None  # stores the current path to follow
        self.status = None
        self.current_goal = None

        str_serv_path = 'global_mission_planner_service'
        print("Waiting for " + str_serv_path)
        rospy.wait_for_service(str_serv_path)
        path_proxy = rospy.ServiceProxy(str_serv_path, ProvidePath)
        self.serv_path = path_proxy(1).local_path
        self.path_numpy = self.pose_array_to_path(self.serv_path)
        print(str_serv_path + " loaded")

        rospy.Subscriber('/tracker/feces_locations', ObjectLocationArray, self.obj_track_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.own_location_callback)
        rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        print("Subscribers and publishers created")

    @staticmethod
    def pose_array_to_path(pose: PoseArray):
        n = len(pose.poses)
        pose_numpy = np.empty((n, 7), dtype=np.float64)

        for i in range(n):
            pose_numpy[i, 0] = pose.poses[i].position.x
            pose_numpy[i, 1] = pose.poses[i].position.y
            pose_numpy[i, 2] = pose.poses[i].position.z
            pose_numpy[i, 3] = pose.poses[i].orientation.x
            pose_numpy[i, 4] = pose.poses[i].orientation.y
            pose_numpy[i, 5] = pose.poses[i].orientation.z
            pose_numpy[i, 6] = pose.poses[i].orientation.w
        return pose_numpy
    
    @staticmethod
    def point_to_array(point: Point):
        arr = np.array([point.x, point.y, point.z])
        return arr
    
    def distance_calc(point):
        return

    def obj_track_callback(self, data: ObjectLocationArray):
        for d in data.object_location:
            d_arr = self.point_to_array(d.abs_location)
            diff = self.path_numpy[:, :3] - d_arr
            dist = np.linalg.norm(diff, ord=2, axis=1)
            min_ind = np.argmin(dist)
            if min_ind == 0:
                insert_ind = 1
            elif min_ind == len(diff) - 1:
                insert_ind = min_ind - 1
            else:
                insert_ind = min_ind if dist[min_ind + 1] < dist[min_ind - 1] else min_ind - 1
            d_arr = np.insert(d_arr, [2], np.zeros_like(4))
            self.path_numpy = np.insert(self.path_numpy, insert_ind, d_arr)
        return

    def own_location_callback(self, data: PoseWithCovarianceStamped):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        self.robot_posn = np.empty((1, 7))
        self.robot_posn[0] = position.x
        self.robot_posn[1] = position.y
        self.robot_posn[2] = position.z
        self.robot_posn[3] = orientation.x
        self.robot_posn[4] = orientation.y
        self.robot_posn[5] = orientation.z
        self.robot_posn[6] = orientation.w
        return

    def battery_callback(self, data: BatteryState):
        self.batter_percentage = data.percentage
        # print(self.batter_percentage)
        if data.percentage < 0.2:
            # Set next goal position as Home
            pass
        pass
