cmake_minimum_required(VERSION 2.8.3)
project(next_best_view_sampling)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS octomap_msgs octomap_ros pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs tf uos_active_perception_msgs visualization_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foo
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs tf uos_active_perception_msgs visualization_msgs
#  DEPENDS libpcl-all-dev 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(next_best_view_node src/active_perception_map.cpp
                                   src/camera_constraints.h
                                   src/next_best_view_node.cpp
                                   src/observation_pose_sampler.cpp
                                   src/octree_ray_iterator.h
                                   src/octree_regions.h)
add_dependencies(next_best_view_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(next_best_view_node
  ${catkin_LIBRARIES}
)

add_executable(nbv_servicetest src/servicetest.cpp)
add_dependencies(nbv_servicetest ${catkin_EXPORTED_TARGETS})
target_link_libraries(nbv_servicetest
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS next_best_view_node nbv_servicetest
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
