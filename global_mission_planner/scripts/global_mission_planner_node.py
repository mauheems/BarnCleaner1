#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from tf.transforms import euler_from_quaternion

class GlobalMissionPlanner:
    def __init__(self):
        rospy.init_node('global_mission_planner_node')

        # Subscriber for the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.robot_amount = int(1)

        # Service for the waypoints
        self.waypoints_service = rospy.Service('global_mission/waypoints', PoseArray, self.waypoints_callback)

        # Placeholder for the map
        self.map_data = None

    def map_callback(self, data):
        # Store the map data
        self.map_data = data

        # Divide the map among robots and generate waypoints
        self.divide_map_and_generate_waypoints()

    def divide_map_and_generate_waypoints(self):
        # Get the map dimensions
        map_width = self.map_data.info.width
        map_height = self.map_data.info.height

        # Calculate the width of each strip
        strip_width = map_width // self.robot_amount

        # Initialize the waypoints list
        waypoints = []

        # Generate waypoints for each robot
        for i in range(self.robot_amount):
            # Calculate the strip boundaries
            strip_start = i * strip_width
            strip_end = (i + 1) * strip_width if i < self.robot_amount - 1 else map_width

            # Initialize the waypoints for this robot
            robot_waypoints = []

            # Generate waypoints in a zigzag pattern
            for y in range(map_height):
                if y % 2 == 0:  # Even rows go from left to right
                    x_range = range(strip_start, strip_end)
                else:  # Odd rows go from right to left
                    x_range = range(strip_end - 1, strip_start - 1, -1)

                for x in x_range:
                    # Create a waypoint at (x, y) and add it to the list
                    waypoint = Pose()
                    waypoint.position.x = x
                    waypoint.position.y = y
                    robot_waypoints.append(waypoint)

            # Add the waypoints for this robot to the main list
            waypoints.append(robot_waypoints)

        return waypoints

    def waypoints_callback(self, request):
        # TODO: Implement the logic to return the waypoints for each robot

        return PoseArray()  # Placeholder

if __name__ == '__main__':
    try:
        GlobalMissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass