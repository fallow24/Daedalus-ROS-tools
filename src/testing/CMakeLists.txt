cmake_minimum_required(VERSION 3.0.2)
project(testing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(orientation_pub orientation_pub.cpp)
target_link_libraries(orientation_pub ${catkin_LIBRARIES})
#add_dependencies(orientation_pub)

add_executable(position_pub position_pub.cpp)
target_link_libraries(position_pub ${catkin_LIBRARIES})
#add_dependencies(position_pub)

add_executable(lidar_pub lidar_pub.cpp)
target_link_libraries(lidar_pub ${catkin_LIBRARIES})
add_dependencies(lidar_pub file_writer)

add_executable(jointstate_pub jointstate_pub.cpp)
target_link_libraries(jointstate_pub ${catkin_LIBRARIES})
