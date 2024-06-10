#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseArray



class GlobalMissionPlanner:
    def __init__(self):
        rospy.init_node('global_mission_planner_node')

        # Subscriber for the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publisher for the waypoints
        self.waypoints_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=10)

        self.grid_pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=10)

        # Placeholder for the map
        self.map_data = None

        rospy.loginfo("Global mission planner node has been initialized")

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

        # Define the block size
        block_size = 10.0  # meters

        # Get the dimensions of the map
        width = self.map_data.info.width
        height = self.map_data.info.height

        # Calculate the number of blocks in the x and y directions
        num_blocks_x = int(width / block_size)
        num_blocks_y = int(height / block_size)

        # Create a 2D array representing the grid
        grid = [[False for _ in range(num_blocks_x)] for _ in range(num_blocks_y)]

        # Iterate over the map data and update the grid
        for y in range(height):
            for x in range(width):
                # Calculate the block indices
                block_x = int(x / block_size)
                block_y = int(y / block_size)

                # Check if the block is available
                if self.map_data.data[y * width + x] == 0:  # 0 represents free space in the map
                    grid[block_y][block_x] = True
                
        # Calculate the scaling factors for x and y coordinates
        scale_x = width / (num_blocks_x * block_size)
        scale_y = height / (num_blocks_y * block_size)
        rospy.loginfo(f'scale factor x: {scale_x}, scale factor y: {scale_y}')

        # Generate a path that covers all available blocks in a snake pattern
        waypoints = PoseArray()
        for y in range(num_blocks_y):
            # Determine the direction of the snake pattern
            if y % 2 == 0:
                # Snake pattern from left to right
                x_range = range(num_blocks_x)
            else:
                # Snake pattern from right to left
                x_range = range(num_blocks_x - 1, -1, -1)

            for x in x_range:
                # Check if the block is available
                if grid[y][x]:
                    # Create a waypoint at the center of the block
                    waypoint = Pose()
                    waypoint.position.x = (x + 0.5) * block_size * scale_x
                    waypoint.position.y = (y + 0.5) * block_size * scale_y
                    waypoints.poses.append(waypoint)

        # Publish the waypoints
        self.publish_waypoints(waypoints)


    def publish_waypoints(self, waypoints):
        waypoints.header.frame_id = "map"
        # Publish the waypoints
        rospy.loginfo(f'Number of waypoints: {len(waypoints.poses)}')
        self.waypoints_pub.publish(waypoints)
        rospy.loginfo("Published waypoints to /waypoints topic")

    def publish_grid(self, grid_msg):
        # Publish the grid
        self.grid_pub.publish(grid_msg)
        rospy.loginfo("Published grid to /grid topic")


if __name__ == '__main__':
    try:
        GlobalMissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



