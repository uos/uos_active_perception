cmake_minimum_required(VERSION 2.8.3)
project(point_head_continuous)

find_package(catkin REQUIRED COMPONENTS actionlib pr2_controllers_msgs roscpp)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib pr2_controllers_msgs roscpp
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(point_head_continuous src/point_head_continuous.cpp)
add_dependencies(point_head_continuous ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_head_continuous
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS point_head_continuous
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
