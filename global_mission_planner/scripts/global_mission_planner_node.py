#!/usr/bin/env python
import time
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseArray, Pose
from mission_planner.srv import ProvidePath, ProvidePathResponse
import math



class GlobalMissionPlanner:
    def __init__(self):
        rospy.init_node('global_mission_planner_node')

        self.waypoints = None

        # Publisher for the waypoints
        self.waypoints_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=10)

        # Subscriber for the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Placeholder for the map
        self.map_data = None


        rospy.loginfo("Global mission planner node has been initialized, waiting for map")

        while self.waypoints is None:
            time.sleep(1)

        # Start service
        str_serv_path = 'global_mission_planner_service'
        self.waypoint_service = rospy.Service(str_serv_path, ProvidePath, self.serve_waypoints)
        rospy.Timer(rospy.Duration(10), self.timer_waypoints)

        rospy.loginfo("Global_mission_planner service started, waypoints provided")

    def timer_waypoints(self, req):
        self.waypoints_pub.publish(self.waypoints)
        return

    def map_callback(self, data):
        # Store the map data
        self.map_data = data
        rospy.loginfo("Map received, generating waypoints...")

        # Divide the map among robots and generate waypoints
        self.divide_map_and_generate_waypoints()

    def divide_map_and_generate_waypoints(self):
        # Check if the map data is available
        if self.map_data is None:
            rospy.logwarn("Map data is not available")
            return

        rospy.loginfo("Now dividing map")

        # Define the block size in terms of cells
        block_size_cells = 2 # number of cells

        # Get the dimensions of the map
        width = self.map_data.info.width
        height = self.map_data.info.height
        rospy.loginfo(f'Map width: {width}, height: {height}')

        # Get the resolution of the map
        resolution = self.map_data.info.resolution
        rospy.loginfo(f'Map resolution: {resolution}')

        width_meters = width * resolution
        height_meters = height * resolution
        rospy.loginfo(f'Map width in meters: {width_meters}, height in meters: {height_meters}')

        # Get the origin of the map
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        rospy.loginfo(f'Map origin: {origin_x, origin_y}')

        # Calculate the block size in meters
        block_size = block_size_cells * resolution
        rospy.loginfo(f'Block size: {block_size}')

        # Calculate the number of blocks in the x and y directions
        num_blocks_x = math.ceil(width / block_size_cells)
        num_blocks_y = math.ceil(height / block_size_cells)
        rospy.loginfo(f'Number of blocks in x and y direction: {num_blocks_x, num_blocks_y}')

        # Create a 2D array representing the grid
        grid = [[False for _ in range(num_blocks_x)] for _ in range(num_blocks_y)]

        # Iterate over the map data and update the grid
        for y in range(4, height - 4):
            for x in range(4, width - 4):
                # Calculate the block indices
                block_x = int(x / block_size_cells)
                block_y = int(y / block_size_cells)

                # Check if the block indices are within the grid dimensions
                if block_x < len(grid[0]) and block_y < len(grid):
                    # Check if the block is available
                    if self.map_data.data[y * width + x] == 0:  # 0 represents free space in the map
                        grid[block_y][block_x] = True

        # Generate a path that covers all available blocks in a snake pattern
        waypoints = PoseArray()
        for y in range(1, num_blocks_y - 1):
            # Determine the direction of the snake pattern
            if y % 2 == 0:
                # Snake pattern from left to right
                x_range = range(1, num_blocks_x - 1)
            else:
                # Snake pattern from right to left
                x_range = range(num_blocks_x - 1, 0, -1)

            for x in x_range:
                # Check if the block is available
                if grid[y][x]:
                    # Create a waypoint at the center of the block
                    waypoint = Pose()
                    waypoint.position.x = (x + 0.5) * block_size + origin_x
                    waypoint.position.y = (y + 0.5) * block_size + origin_y
                    waypoints.poses.append(waypoint)

        # Make the last waypoint the same as the first waypoint
        if waypoints.poses:
            waypoints.poses.append(waypoints.poses[0])

        rospy.loginfo(f'First few waypoints: {waypoints.poses[:5]}')

        # Publish the waypoints
        waypoints.header.frame_id = "map"


        self.waypoints_pub.publish(waypoints)
        self.waypoints = waypoints
        return


    def publish_waypoints(self, waypoints):
        # Publish the waypoints
        rospy.loginfo(f'Number of waypoints: {len(waypoints.poses)}')
        self.waypoints_pub.publish(waypoints)
        rospy.loginfo("Published waypoints to /waypoints topic")

    def publish_grid(self, grid_msg):
        # Publish the grid
        self.grid_pub.publish(grid_msg)
        rospy.loginfo("Published grid to /grid topic")

    def serve_waypoints(self, req):
        return self.waypoints


if __name__ == '__main__':
    try:
        GlobalMissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



