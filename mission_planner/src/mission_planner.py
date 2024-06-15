import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point
from custom_msgs.msg import ObjectLocationArray, ObjectLocation
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from mission_planner.srv import *
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseActionResult, MoveBaseGoal
import actionlib
import numpy as np
import time

def dict_to_pose(pose_dict: dict) -> Pose:
    """
    Convert a dictionary to a geometry_msgs/Pose object.

    :param pose_dict: Dictionary containing pose information
    :type pose_dict: dict
    :return: Pose object
    :rtype: geometry_msgs.msg.Pose
    """
    pose = Pose()

    pose.position.x = pose_dict['position']['x']
    pose.position.y = pose_dict['position']['y']
    pose.position.z = pose_dict['position']['z']
    pose.orientation.x = pose_dict['orientation']['x']
    pose.orientation.y = pose_dict['orientation']['y']
    pose.orientation.z = pose_dict['orientation']['z']
    pose.orientation.w = pose_dict['orientation']['w']
    return pose


class MissionPlanner:
    def __init__(self):
        """
        Initializes the MissionPlanner node.

        This function initializes the MissionPlanner node by setting up the necessary
        variables and services. It creates the following variables:
        - `robot_id`: an integer representing the ID of the robot.
        - `path_numpy`: a NumPy array representing the path of the robot.
        - `faeces_location`: a list representing the location of feces.
        - `robot_posn`: a variable representing the position of the robot.
        - `status`: a variable representing the status of the robot.
        - `current_goal_id`: an integer representing the ID of the current goal.
        - `last_move_time`: a float representing the time of the last move.

        It also sets up the following services:
        - `path_proxy`: a service proxy for the global mission planner service.
        - `move_base_client`: an action client for the move base service.

        The function waits for the global mission planner service and the move base
        server to be available. If the uniform distribution option is selected, it
        resets the 2D pose using the global localization service. Otherwise, it sets
        the location to the home position.

        The function subscribes to the `/tracker/feces_locations` topic to track
        feces locations and sets up a timer to update the goal every 0.1 seconds.

        This function does not take any parameters and does not return anything.
        """
        print("MissionPlanner node initialized")
        self.robot_id : int = None
        self.path_numpy: np.ndarray = None
        self.faeces_location: list = []
        self.robot_posn = None
        self.status = None
        self.current_goal_id = None
        self.last_move_time = time.time()

        str_serv_path = 'global_mission_planner_service'
        print("Waiting for " + str_serv_path)
        rospy.wait_for_service(str_serv_path)
        self.path_proxy = rospy.ServiceProxy(str_serv_path, ProvidePath)
        self.update_path(Bool(data=True))
        print(str_serv_path + " loaded")

        print("Waiting for move base server")
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        print("Move pase server loaded")

        if True:  # Uniform distribution of current position across the map
            gloabl_localization_service = '/global_localization'
            print("Waiting for AMCL " + gloabl_localization_service)
            rospy.wait_for_service(gloabl_localization_service)
            self.localization_reset = rospy.ServiceProxy(gloabl_localization_service, Empty)
            print("Service " + gloabl_localization_service + " loaded. Resetting 2D pose")
            self.localization_reset()
        else:  # Set location to home position
            # Publish to topic /initialpose
            pass

        self.move_base_client.send_goal(self.pose_numpy_to_goal_pose(self.path_numpy[self.current_goal_id]))


        rospy.Subscriber('/tracker/feces_locations', ObjectLocationArray, self.obj_track_callback)
        # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.own_location_callback)
        # rospy.Subscriber('/mirte/power/power_watcher', BatteryState, self.battery_callback)
        # rospy.Subscriber('/mission_planner/reset', Bool, self.update_path)

        rospy.Timer(rospy.Duration(secs=0.1), self.goal_update)

        print("Subscribers and publishers created")

    def update_path(self, msg: Bool):
        """
        Updates the path of the robot based on the provided message.

        Args:
            msg (Bool): The message indicating whether to update the path.

        Returns:
            None

        This function calls the `path_proxy` service to get the local path from the global mission planner service.
        It then converts the pose array to a NumPy array using the `pose_array_to_path` function.
        The `current_goal_id` is set to 0 to indicate the start of the path.
        """
        serv_path = self.path_proxy(1).local_path
        self.path_numpy = self.pose_array_to_path(serv_path)
        self.current_goal_id = 0
        return
    
    def goal_update(self, event):
        """
        Updates the current goal based on the state of the move_base client.

        Args:
            event (rospy.TimerEvent): The event object representing the timer event.

        Returns:
            None

        This function checks the state of the move_base client and updates the current goal accordingly. If the state is 3, it logs a message indicating that the goal has been reached and moves to the next goal. If the state is 4, it logs an error message indicating that the goal could not be reached and moves to the next goal. If the state is 1, it does nothing as the robot is already going to the goal. If the state is any other value, it logs an error message indicating the encountered state.
        """
        state = self.move_base_client.get_state()
        if state == 3:
            rospy.loginfo("Reached goal, going to next goal")
            self.current_goal_id += 1
            self.move_base_client.send_goal(self.pose_numpy_to_goal_pose(self.path_numpy[self.current_goal_id]))
        elif state == 4:
            rospy.logerr("Unable to reach goal, moving on")
            self.current_goal_id += 1
            self.move_base_client.send_goal(self.pose_numpy_to_goal_pose(self.path_numpy[self.current_goal_id]))
        elif state == 1:
            pass  # Robot is going to the goal, do nothing
        else:
            rospy.logerr("This state was encountered: " + str(state))
        return

    @staticmethod
    def pose_array_to_path(pose: PoseArray):
        """
        Converts a PoseArray message to a numpy array representing a path.

        Args:
            pose (PoseArray): The PoseArray message to convert.

        Returns:
            numpy.ndarray: A numpy array of shape (n, 7) representing the path.
                Each row contains the x, y, z, qx, qy, qz, and qw components of a pose.

        This function converts a PoseArray message to a numpy array representing a path.
        It iterates over each pose in the message and extracts the x, y, and orientation components.
        The resulting numpy array has shape (n, 7), where n is the number of poses in the message.
        """
        n = len(pose.poses)
        pose_numpy = np.empty((n, 7), dtype=np.float64)

        for i in range(n):
            pose_numpy[i, 0] = pose.poses[i].position.x
            pose_numpy[i, 1] = pose.poses[i].position.y
            pose_numpy[i, 2] = 0
            pose_numpy[i, 3] = 0
            pose_numpy[i, 4] = 0
            pose_numpy[i, 5] = 0.8937268896831463
            pose_numpy[i, 6] = 0.44861146514248745
        return pose_numpy
    
    @staticmethod
    def pose_numpy_to_goal_pose(arr):
        """
        Convert a numpy array to a MoveBaseGoal message representing a goal pose.

        Args:
            arr (numpy.ndarray): A numpy array of shape (7,) representing the pose.
                The array contains the x, y, z, qx, qy, qz, and qw components of a pose.

        Returns:
            MoveBaseGoal: A MoveBaseGoal message representing the goal pose.
                The message contains the target pose with the specified x, y, z, and orientation components.
                The frame_id of the target pose is set to 'map'.
                The stamp of the target pose is set to the current time.

        This function converts a numpy array to a MoveBaseGoal message representing a goal pose.
        It creates a Pose message with the specified x, y, and orientation components.
        The resulting MoveBaseGoal message contains the target pose with the specified x, y, z, and orientation components.
        The frame_id of the target pose is set to 'map'.
        The stamp of the target pose is set to the current time.
        """
        goal = MoveBaseGoal()
        pose = Pose()
        pose.position.x = arr[0]
        pose.position.y = arr[1]
        pose.position.z = arr[2]
        pose.orientation.x = arr[3]
        pose.orientation.y = arr[4]
        pose.orientation.z = arr[5]
        pose.orientation.w = arr[6]
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        return goal
    
    @staticmethod
    def point_to_array(point: Point):
        arr = np.array([point.x, point.y, 0])
        return arr
    
    def distance_calc(point):
        return

    def obj_track_callback(self, data: ObjectLocationArray):
        """
        This function tracks objects based on their location data provided in the ObjectLocationArray.
        It calculates the closest object in the path, determines the insertion index, and updates the path accordingly.
        """
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
        rospy.loginfo("own_location_callback")
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        old_posn = self.robot_posn
        self.robot_posn = np.empty((1, 7))
        self.robot_posn[0, 0] = position.x
        self.robot_posn[0, 1] = position.y
        self.robot_posn[0, 2] = position.z
        self.robot_posn[0, 3] = orientation.x
        self.robot_posn[0, 4] = orientation.y
        self.robot_posn[0, 5] = orientation.z
        self.robot_posn[0, 6] = orientation.w
        if old_posn is not None:
            dist = np.linalg.norm(old_posn[0, :3] - self.robot_posn[0, :3], 2)
            if dist > 0.1:
                self.last_move_time = time.time()       
            elif time.time() - self.last_move_time > 300:
                pass # Move to next goal
            else:
                pass # Do nothing
        else:
            pass
        return

    def battery_callback(self, data: BatteryState):
        self.batter_percentage = data.percentage
        # print(self.batter_percentage)
        if data.percentage < 0.2:
            # Set next goal position as Home
            pass
        pass
