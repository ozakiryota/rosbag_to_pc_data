cmake_minimum_required(VERSION 3.0.2)
project(rosbag_to_pc_data)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(catkin REQUIRED
    roscpp
    rosbag
)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(rosbag_to_pcd
    src/rosbag_to_pcd.cpp
    src/load_rosbag.cpp
)
target_link_libraries(rosbag_to_pcd
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(rosbag_to_ply
    src/rosbag_to_ply.cpp
    src/load_rosbag.cpp
)
target_link_libraries(rosbag_to_ply
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
