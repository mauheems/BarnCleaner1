cmake_minimum_required(VERSION 3.0.2)
project(hri_communication)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  std_msgs
  message_generation  # Add this line
)

add_service_files(
  FILES
  StartMapping.srv
  StartNavigation.srv
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roslaunch rospy std_msgs message_runtime  # Add message_runtime here
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)