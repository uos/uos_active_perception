cmake_minimum_required(VERSION 2.8.3)
project(sensor_motion_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS sensor_msgs tf
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(point_cloud_2_motion_filter src/point_cloud_2_motion_filter.cpp)
add_dependencies(point_cloud_2_motion_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_2_motion_filter
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS point_cloud_2_motion_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

