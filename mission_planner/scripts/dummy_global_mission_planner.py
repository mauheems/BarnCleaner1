from __future__ import print_function
import rospy
from geometry_msgs.msg import Pose, PoseArray
from mission_planner.srv import ProvidePath, ProvidePathResponse
import re


# Define the function to extract position and orientation from a given text chunk
def extract_position_orientation(text_chunk):
    # Regular expressions to extract position and orientation
    position_pattern = re.compile(r'position:\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)')
    orientation_pattern = re.compile(r'orientation:\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)\s*w:\s*([-\d.]+)')

    # Search for patterns in the text
    position_match = position_pattern.search(text_chunk)
    orientation_match = orientation_pattern.search(text_chunk)

    # Extract values if matches are found
    if position_match:
        pos_x, pos_y, pos_z = position_match.groups()
        print(f"Position -> x: {pos_x}, y: {pos_y}, z: {pos_z}")

    if orientation_match:
        orient_x, orient_y, orient_z, orient_w = orientation_match.groups()
        print(f"Orientation -> x: {orient_x}, y: {orient_y}, z: {orient_z}, w: {orient_w}")

    pose = Pose()
    pose.position.x = float(pos_x)
    pose.position.y = float(pos_y)
    pose.position.z = float(pos_z)
    pose.orientation.x = float(orient_x)
    pose.orientation.y = float(orient_y)
    pose.orientation.z = float(orient_z)
    pose.orientation.w = float(orient_w)
    return pose

    

# Load text from a file in chunks of 30 lines
def load_text_in_chunks(file_path, chunk_size=17):
    with open(file_path, 'r') as file:
        lines = []
        for line in file:
            lines.append(line)
            if len(lines) == chunk_size:
                yield ''.join(lines)
                lines = []
        # Yield any remaining lines
        if lines:
            yield ''.join(lines)

def provide_path(req):
    return ProvidePathResponse(pose_array)
    

def global_mission_planner_service():
    str_serv_path = 'global_mission_planner_service'
    rospy.init_node("GMP_Service")
    s = rospy.Service(str_serv_path, ProvidePath, provide_path)
    print("Ready to provide path")
    rospy.spin()

if __name__ == "__main__":
    pose_array = PoseArray()
    poses = []
    for value in load_text_in_chunks('/home/mirte/mirte_ws/goals.txt'):
        poses.append(extract_position_orientation(value))
    pose_array.poses = poses
    global_mission_planner_service()
