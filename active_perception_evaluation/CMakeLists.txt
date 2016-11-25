cmake_minimum_required(VERSION 2.8.3)
project(active_perception_evaluation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  gazebo_msgs
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(floating_kinect_localizer src/floating_kinect_localizer.cpp)
add_dependencies(floating_kinect_localizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(floating_kinect_localizer
  ${catkin_LIBRARIES}
)

add_executable(sim_amcl_update src/sim_amcl_update.cpp src/sim_manipulator.cpp)
add_dependencies(sim_amcl_update ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_amcl_update
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/common.py
  scripts/eval_plans.py
  scripts/eval_runs.py
  scripts/evaluate_random_drive.py
  scripts/evaluate_searchrun.py
  scripts/evman.py
  scripts/plot_nbv_coverage.py
  scripts/time_comparison.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS floating_kinect_localizer sim_amcl_update
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
