cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(object_search_manager src/object_search_manager.h
                                              src/object_search_manager_node.cpp
                                              src/pr2_agent.cpp
                                              src/floating_kinect_agent.cpp
                                              src/observation_pose_collection.cpp
                                              src/kmeans.cpp
                                              src/ros_serialization_helper.h
                                              src/geometry.h
                                              src/search_plan.h
                                              src/search_planner.h)
rosbuild_add_executable(object_search_manager_test src/object_search_manager_test.cpp)
rosbuild_add_executable(kmeans-test src/kmeans.cpp
                                    src/kmeans-test.cpp)
rosbuild_add_executable(random_driver src/random_driver.cpp)
rosbuild_add_executable(nbv_sampling_evaluator src/nbv_sampling_evaluator.cpp
                                               src/observation_pose_collection.cpp)
