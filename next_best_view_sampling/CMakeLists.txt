cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(ENV{octomap_DIR} "$ENV{ROS_WORKSPACE}/libs/octomap/octomap/lib/cmake/octomap")
find_package(octomap 1.6 REQUIRED)
include_directories(BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARY_DIRS}/liboctomap.so ${OCTOMAP_LIBRARY_DIRS}/liboctomath.so)
MESSAGE(STATUS "OctoMap paths:" ${OCTOMAP_INCLUDE_DIRS} " " ${OCTOMAP_LIBRARY_DIRS})

rosbuild_add_executable(next_best_view_node src/active_perception_map.cpp
                                            src/next_best_view_node.cpp
                                            src/camera_constraints.h
                                            src/octree_regions.h
                                            src/octree_ray_iterator.h
                                            src/observation_pose_sampler.cpp)

rosbuild_add_executable(servicetest src/servicetest.cpp)
