cmake_minimum_required(VERSION 3.0.2)
project(A1_base)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  geometry_msgs
  sensor_msgs
  tf
  trajectory_msgs
  genmsg
  roscpp
  unitree_legged_msgs
  jsk_recognition_msgs
  kindr
  kindr_ros
  grid_map_ros
  grid_map_msgs
)

find_package(Eigen3 REQUIRED)

# find_package(CGAL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    unitree_legged_msgs
    grid_map_core
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# add_dependencies(controller_node unitree_legged_msgs_gencpp)
add_library(${PROJECT_NAME}_library
  src/a1_adapter.cpp
  src/body_controller.cpp 
  src/controller.cpp
  src/foot_trajectory_planner.cpp
  src/kinematics.cpp
  src/leg_controller.cpp 
  src/phase_signal_generator.cpp 
  src/QuadProg++.cc
  src/util_transformations.cpp
)

add_executable(controller_node src/controller_node.cpp src/pose_optimizer.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} ${EXTRA_LIBS} ${PROJECT_NAME}_library)