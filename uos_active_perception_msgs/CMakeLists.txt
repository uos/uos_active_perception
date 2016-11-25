cmake_minimum_required(VERSION 2.8.3)
project(uos_active_perception_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoundingBox.msg
  CellId.msg
  CellIds.msg
  ConditionalVisibilityMap.msg
  ObjectSet.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  EvaluateObservationCameraPoses.srv
  GetBboxOccupancy.srv
  GetObservationCameraPoses.srv
  ReadWriteFiles.srv
  ResetVolumes.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tmp
  CATKIN_DEPENDS geometry_msgs message_runtime
#  DEPENDS system_lib
)
