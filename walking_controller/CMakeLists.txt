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

# link_directories(
# 	$ENV{UNITREE_LEGGED_SDK_PATH}/lib 
# )

# set(EXTRA_LIBS libunitree_legged_sdk_arm64.so lcm)

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
  src/real_robot.cpp
  src/pose_optimizer.cpp
)

include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_arm64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)
# add_definitions(-DSDK3_2)

#add_executable(controller_node src/controller_node.cpp)
#target_link_libraries(controller_node ${catkin_LIBRARIES} ${EXTRA_LIBS} ${PROJECT_NAME}_library)

#add_executable(lowstate_pub src/lowstate_pub/lowstate_pub.cpp)
#target_link_libraries(lowstate_pub ${catkin_LIBRARIES} ${EXTRA_LIBS})

# include_directories(/home/kangsinwon/unitree_legged_sdk/include)
# link_directories(/home/kangsinwon/unitree_legged_sdk/lib)
# string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)

# set_target_properties( unitree_sdk PROPERTIES IMPORTED_LOCATION /home/kangsinwon/unitree_legged_sdk/lib/libunitree_legged_sdk_amd64.so )

add_executable(real_bot src/real_bot.cpp)
target_link_libraries(real_bot ${catkin_LIBRARIES} ${EXTRA_LIBS} ${PROJECT_NAME}_library)
add_dependencies(real_bot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
