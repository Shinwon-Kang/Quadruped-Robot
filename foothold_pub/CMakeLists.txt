cmake_minimum_required(VERSION 3.0.2)
project(foothold_pub)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  # grid_map_core
  # grid_map_ros
  # grid_map_msgs
  champ_msgs
)

catkin_package(
#  CATKIN_DEPENDS
#  LIBRARIES champ_msgs
#  LIBRARIES foothold_pub
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(foothold_pub src/foothold_pub_node.cpp 
                            # src/foothold_pub.cpp src/foothold_map.cpp)

# target_link_libraries(foothold_pub
  # ${catkin_LIBRARIES}
# )
