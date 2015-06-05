uos_active_perception
=====================

This is a ROS stack for code of the project FLAP for CAOS (Forward-Looking Active Perception for Clutter-Aware Object
Search). All subdirectories are ROS packages.

How to install the system and its dependencies into an empty workspace on ROS Fuerte
------------------------------------------------------------------------------------

1. Add the following lines to your .rosinstall:

        # The uos_active_perception repo
        - git: {local-name: uos_active_perception, uri: 'software@kure.informatik.uos.de:uos_active_perception', version: fuerte}
        # A fork of the ROS Navigation Stack featuring symmetrical path costs and batch path planning
        - git: {local-name: overlays/navigation, uri: 'https://github.com/uos/navigation.git', version: fuerte_race}
        # Octomap 1.6 (ROS Fuerte comes with version 1.4, which is too old)
        # NOTE: The path libs/octomap is hard-coded in uos_active_perception/next_best_view_sampling/CMakeLists.txt.
        #       This means you do not need to install octomap, but if this path changes it needs to be manually adjusted.
        - git: {local-name: libs/octomap, uri: 'https://github.com/OctoMap/octomap.git', version: v1.6.6}
        # PCL
        - git: {local-name: stacks/pcl_conversions, uri: 'https://github.com/ros-perception/pcl_conversions.git', version: fuerte-devel}
        - git: {local-name: stacks/pcl_msgs, uri: 'https://github.com/ros-perception/pcl_msgs', version: fuerte-devel}

2. Update the workspace

        rosws update
        source $ROS_WORKSPACE/setup.bash

3. Compile octomap

        cd $ROS_WORKSPACE/libs/octomap/octomap
        mkdir -p build && cd build
        cmake ..
        make
        
4. Compile all ROS packages

        rosmake --all
  
How to run the system with Gazebo
---------------------------------

### Using the PR2 robot

Execute each of the following steps in a new terminal:

1. Start Gazebo

        roslaunch active_perception_evaluation race_world.launch gui:=true

2. Spawn objects of the demo environment and the robot

        roslaunch active_perception_evaluation spawn_race_objects.launch
        roslaunch active_perception_evaluation spawn_pr2.launch

3. Start the simulation; tuck PR2's arms; start AMCL robot localiazation

        rosservice call /gazebo/unpause_physics
        rosrun pr2_tuckarm tuck_arms.py -l t -r t -q
        roslaunch active_perception_evaluation pr2_navigation.launch

4. Launch RViz

        rosrun rviz rviz -d $(rospack find race_object_search)/config/object_search.vcg

5. Start mapping and view sampling

        roslaunch race_object_search object_search_prerequisites_pr2.launch sim:=true map:=race_extended_inflated fk:=false

6. Start object search using default planner parameters

        roslaunch race_object_search object_search_manager.launch robot:=pr2

7. Execute object search with 6 target volumes, each with 20% success probability and 5% termination threshold

        rosrun race_object_search object_search_manager_test p table1 0.2 table2 0.2 counter 0.2 shelf1 0.2 shelf2 0.2 shelf3 0.2 min_p_succ 0.05


### Using the simplified PR2 (floating kinect)

Execute each of the following steps in a new terminal:

1. Start Gazebo

        roslaunch active_perception_evaluation race_world.launch gui:=true
        
2. Spawn objects of the demo environment and the sensor

        roslaunch active_perception_evaluation spawn_race_objects.launch
        roslaunch active_perception_evaluation spawn_floating_kinect.launch
        
3. Start the simulation; start ground truth robot localiazation

        rosservice call /gazebo/unpause_physics
        roslaunch active_perception_evaluation floating_kinect_navigation.launch
        
4. Launch RViz

        rosrun rviz rviz -d $(rospack find race_object_search)/config/object_search.vcg
        
5. Start mapping and view sampling

        roslaunch race_object_search object_search_prerequisites_pr2.launch sim:=true map:=race_extended_inflated fk:=true
        
6. Start object search using default planner parameters

        roslaunch race_object_search object_search_manager.launch robot:=floating_kinect
        
7. Execute object search with 6 target volumes, each with 20% success probability and 5% termination threshold

        rosrun race_object_search object_search_manager_test p table1 0.2 table2 0.2 counter 0.2 shelf1 0.2 shelf2 0.2 shelf3 0.2 min_p_succ 0.05


How to run automatic evaluation
-------------------------------

1. cd into empty directory (lots of log output will be placed here)
2. Without any ROS instance running, start the evaluation

        rosrun active_perception_evaluation evman.py "thesis_nomap(N=20, gui=True)"

   * Available experiments: `thesis_nomap`, `thesis_withmap`
   * `N=n` adjusts the number of trials
   * `gui=False` for headless execution
3. Evaluate generated logs
   * cd into experiment subfolder
   * To evaluate and compare initial plans for all planner configurations:

            rosrun active_perception_evaluation eval_plans.py

   * To evaluate and compare full search runs for all planner configurations:

            rosrun active_perception_evaluation eval_runs.py ()

   * To graphically compare two search runs (e.g., `greedy_0` and `iw10_0`):

            rosrun active_perception_evaluation evaluate_searchrun.py greedy_0 iw10_0
