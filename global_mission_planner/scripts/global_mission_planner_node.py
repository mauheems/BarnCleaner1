#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion

class GlobalMissionPlanner:
    def __init__(self):
        rospy.init_node('global_mission_planner_node')

        # Subscriber for the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

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
        # Check if the map data is available
        if self.map_data is None:
            rospy.logwarn("Map data is not available")
            return

        # Get the dimensions of the map
        width = self.map_data.info.width
        height = self.map_data.info.height

        # Calculate the size of each partition
        partition_width = width // 3
        partition_height = height

        # Generate waypoints for each partition
        for i in range(3):
            # Calculate the starting and ending indices for the partition
            start_index = i * partition_width
            end_index = (i + 1) * partition_width

            # Create a new PoseArray for the waypoints
            waypoints = PoseArray()

            # Determine the direction of the snake pattern
            if i % 2 == 0:
                # Snake pattern from left to right
                x_range = range(start_index, end_index)
            else:
                # Snake pattern from right to left
                x_range = range(end_index - 1, start_index - 1, -1)

            # Iterate over the map data within the partition
            for y in range(partition_height):
                for x in x_range:
                    # TODO: Generate waypoints based on the map data at (x, y)

                    # Example: Create a waypoint at (x, y)
                    waypoint = Pose()
                    waypoint.position.x = x
                    waypoint.position.y = y
                    waypoints.poses.append(waypoint)

            # Publish the waypoints for the current partition
            self.publish_waypoints(waypoints)

    def publish_waypoints(self, waypoints):
        # TODO: Publish the waypoints for the current partition
        # Create a publisher for the waypoints
        waypoints_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=10)
        # Publish the waypoints
        waypoints_pub.publish(waypoints)

if __name__ == '__main__':
    try:
        GlobalMissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    