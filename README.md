uos_active_perception
======================

This is a ROS stack for our amazing project code. All subdirectories are ROS
packages.

How to launch the system on the real PR2
----------------------------------------

    roslaunch race_bringup race_prerequisites.launch sim:=false
    roslaunch next_best_view_sampling next_best_view_pr2_real.launch
    rosrun race_object_search object_search_manager _world_frame_id:=/map
    rosrun race_object_search object_search_manager_test


How to launch the system in Gazebo
----------------------------------

### prerequisites

    roslaunch race_gazebo_worlds race_world_pr2_simrun.launch demo_21a:=true
    roslaunch race_bringup race_prerequisites.launch sim:=true
    rosrun pr2_tuckarm tuck_arms.py b


### octomap mapping and NBV sampling

    roslaunch next_best_view_sampling next_best_view_pr2.launch


### object search action server

    rosrun race_object_search object_search_manager _world_frame_id:=/map


### test service (explores 1m^3 box without stop criterium)

    rosrun race_object_search object_search_manager_test

The octomap, including fringe voxels and generated samples are published on the /next_best_view_marker topic.
