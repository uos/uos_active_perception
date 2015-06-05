/*
 * 2013-08-26 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 * Updates the AMCL pose hypothesis by querying the Gazebo simulation for the
 * robot's ground truth.
 */
#include "ros/ros.h"
#include "sim_manipulator.h"

int main(int argc, char **argv)
{
  std::string pr2_gazebo_name;
  if(argc > 1)
    pr2_gazebo_name = argv[1];
  else
    ROS_ERROR("Need robot model name as cli argument");

  ros::init(argc, argv, "sim_amcl_update");
  ros::NodeHandle nh;

  // Set Gazebo to RACE map transform
  SimManipulator simManipulator(nh,
    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-12.28, -10.20, 0.0)));

  if (argc >= 2)
    pr2_gazebo_name = argv[1];

  // Update the AMCL localization with the pose from the simulation
  simManipulator.updateAmclFromSimulation(pr2_gazebo_name);

  return 0;
}
