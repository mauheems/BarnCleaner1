cmake_minimum_required(VERSION 3.0.2)
project(global_mission_planner)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/global_mission_planner_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)