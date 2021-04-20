cmake_minimum_required(VERSION 3.0.2)
project(A1_base)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  Eigen3
  geometry_msgs
  sensor_msgs
  tf
  trajectory_msgs
  genmsg
  unitree_legged_msgs
  roscpp
)

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
#    CATKIN_DEPENDS 
#    unitree_legged_msgs 
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

add_executable(controller_node src/controller_node.cpp src/controller.cpp 
                      src/body_controller.cpp src/leg_controller.cpp src/kinematics.cpp
                      src/phase_signal_generator.cpp src/foot_trajectory_planner.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} ${EXTRA_LIBS})