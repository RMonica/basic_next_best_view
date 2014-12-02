cmake_minimum_required(VERSION 2.8.3)
project(basic_next_best_view)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  message_generation
  eigen_conversions
  cmake_modules
)

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  VoxelFilter.msg
  BasicRequest.msg
  BasicResponse.msg
  )

add_action_files(
  DIRECTORY
    action
  FILES
    Execute.action
  )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp sensor_msgs std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(basic_next_best_view
  src/basic_next_best_view.cpp
  src/basic_viewpoint_evaluator.cpp
  src/RayTracer.cpp
  )

add_dependencies(basic_next_best_view 
  basic_next_best_view_generate_messages_cpp
  )

target_link_libraries(basic_next_best_view
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  )

#############
## Install ##
#############

#############
## Testing ##
#############

