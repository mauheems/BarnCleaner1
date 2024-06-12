#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseArray
import math
from visualization_msgs.msg import Marker, MarkerArray



class GlobalMissionPlanner:
    def __init__(self):
        rospy.init_node('global_mission_planner_node')

        # Subscriber for the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publisher for the waypoints
        self.waypoints_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=10)

        # Placeholder for the map
        self.map_data = None

        rospy.loginfo("Global mission planner node has been initialized")

        # Initialize the publisher
        self.grid_pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=10)

        # Publisher for the markers
        self.marker_pub = rospy.Publisher('/markers', MarkerArray, queue_size=10)

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
        num_blocks_x = width // block_size_cells
        num_blocks_y = height // block_size_cells
        rospy.loginfo(f'Number of blocks in x and y direction: {num_blocks_x, num_blocks_y}')

        # Create a 2D array representing the grid
        grid = [[False for _ in range(num_blocks_x)] for _ in range(num_blocks_y)]

        # Iterate over the map data and update the grid
        for y in range(height):
            for x in range(width):
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
                x_range = range(num_blocks_x - 2, 0, -1)

            for x in x_range:
                # Check if the block is available
                if grid[y][x]:
                    # Create a waypoint at the center of the block
                    waypoint = Pose()
                    waypoint.position.x = (x + 0.5) * block_size + origin_x
                    waypoint.position.y = (y + 0.5) * block_size + origin_y
                    waypoints.poses.append(waypoint)

        # Publish the markers
        self.publish_markers(waypoints)

        # Make the last waypoint the same as the first waypoint
        if waypoints.poses:
            waypoints.poses.append(waypoints.poses[0])

        rospy.loginfo(f'First few waypoints: {waypoints.poses[:5]}')

        # Publish the waypoints
        self.publish_waypoints(waypoints)

        # publish the grid
        self.publish_grid(grid_msg=grid)


    def publish_waypoints(self, waypoints):
        waypoints.header.frame_id = "map"
        # Publish the waypoints
        rospy.loginfo(f'Number of waypoints: {len(waypoints.poses)}')
        self.waypoints_pub.publish(waypoints)
        rospy.loginfo("Published waypoints to /waypoints topic")

    

    def publish_grid(self, grid_msg):
        # Convert the grid to a numpy array
        grid_array = np.array(grid_msg)

        # Convert the boolean grid to occupancy values [0, 100]
        grid_array = np.where(grid_array, np.uint8(0), np.uint8(100))

        # Create an OccupancyGrid message
        map_msg = OccupancyGrid()

        # Set the header information
        # map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"  # Change this to your map's frame_id

        # Set the map metadata (size and resolution)
        map_msg.info.width = grid_array.shape[1]
        map_msg.info.height = grid_array.shape[0]
        map_msg.info.resolution = 1  # Change this to your map's resolution

        # Flatten the grid array and convert it to a list
        map_msg.data = grid_array.flatten().tolist()
        # print(grid_array)
        print(map_msg.data)

        # Publish the map
        self.grid_pub.publish(map_msg)
        rospy.loginfo("Published grid to /grid topic")

    def publish_markers(self, waypoints):
        # Create a MarkerArray message
        marker_array = MarkerArray()
        # Iterate over the waypoints and create a marker for each one
        for i, waypoint in enumerate(waypoints.poses):
            # Create a Marker message
            marker = Marker()
            # Set the marker properties
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose = waypoint
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # point_1 = Point(waypoint.position.x-1,waypoint.position.y+1, 0)
            # point_2 = Point(waypoint.position.x+1,waypoint.position.y+1, 0)
            # point_3 = Point(waypoint.position.x+1,waypoint.position.y-1, 0)
            # point_4 = Point(waypoint.position.x-1,waypoint.position.y-1, 0)

            point_1 = Point(-1,+1, 0)
            point_2 = Point(+1,+1, 0)
            point_3 = Point(+1,-1, 0)
            point_4 = Point(-1,-1, 0)

            marker.points = [point_1, point_2, point_3, point_4, point_1]
            # Add the marker to the marker array
            marker_array.markers.append(marker)
        # Publish the marker array
        self.marker_pub.publish(marker_array)
        rospy.loginfo("Published markers to /markers topic")

    


if __name__ == '__main__':
    try:
        GlobalMissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



