cmake_minimum_required(VERSION 2.8.3)
project(race_object_search)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  gazebo_msgs
  message_generation
  move_base
  nav_msgs
  next_best_view_sampling
  pr2_controllers_msgs
  uos_active_perception_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  FILES
  ObserveVolumes.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  uos_active_perception_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
  actionlib
  actionlib_msgs
  gazebo_msgs
  message_runtime
  move_base
  nav_msgs
  next_best_view_sampling
  pr2_controllers_msgs
  uos_active_perception_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(object_search_manager src/floating_kinect_agent.cpp
                                     src/geometry.h
                                     src/kmeans.cpp
                                     src/object_search_manager.h
                                     src/object_search_manager_node.cpp
                                     src/observation_pose_collection.cpp
                                     src/pr2_agent.cpp
                                     src/ros_serialization_helper.h
                                     src/search_plan.h
                                     src/search_planner.h)
add_dependencies(object_search_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_search_manager
  ${catkin_LIBRARIES}
)

add_executable(object_search_manager_test src/object_search_manager_test.cpp)
add_dependencies(object_search_manager_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_search_manager_test
  ${catkin_LIBRARIES}
)

add_executable(kmeans-test src/kmeans.cpp
                           src/kmeans-test.cpp)
add_dependencies(kmeans-test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kmeans-test
  ${catkin_LIBRARIES}
)

add_executable(random_driver src/random_driver.cpp)
add_dependencies(random_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(random_driver
  ${catkin_LIBRARIES}
)

add_executable(nbv_sampling_evaluator src/floating_kinect_agent.cpp
                                      src/kmeans.cpp
                                      src/nbv_sampling_evaluator.cpp
                                      src/observation_pose_collection.cpp
                                      src/pr2_agent.cpp)
add_dependencies(nbv_sampling_evaluator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nbv_sampling_evaluator
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS kmeans-test nbv_sampling_evaluator object_search_manager object_search_manager_test random_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
