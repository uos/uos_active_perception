#ifndef SIM_MANIPULATOR_H
#define SIM_MANIPULATOR_H

#include "ros/ros.h"
#include "ros/package.h"
#include "tf/tf.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <stdio.h>

class SimManipulator
{
protected:
  ros::NodeHandle nh_;
  // ROS Service handles
  std::map<std::string, ros::ServiceClient> service_clients;
  // Map to Gazebo transformation
  tf::Transform map_2_gazebo;

public:
  SimManipulator( void );
  SimManipulator(const ros::NodeHandle & nh);
  SimManipulator(const ros::NodeHandle & nh, const tf::Transform & map_2_gazebo);
  ~SimManipulator(void);
  tf::Transform getMap2GazeboTransform(void);
  bool moveObject (const gazebo_msgs::ModelState & modelstate);
  bool waitForService(ros::ServiceClient & serv_client, uint32_t wait_sec);
  bool pause (void);
  bool unpause (void);
  bool resetSimulation (void);
  bool resetWorld (void);
  std::vector<gazebo_msgs::ModelState> getModelPoses (void);
  bool getModelPose(gazebo_msgs::ModelState & pose);
  bool modelExistsInSimulation(const std::string & model_name);
  bool setAmclPose(gazebo_msgs::ModelState & pose);
  bool updateAmclFromSimulation (const std::string & gazebo_name);
  bool setUpdateRate (const float & update_rate);
  gazebo_msgs::GetPhysicsProperties getPhysicsProperties (void);
};

#endif // SIM_MANIPULATOR_H
