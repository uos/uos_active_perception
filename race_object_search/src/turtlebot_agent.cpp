#include "turtlebot_agent.h"

#include "kmeans.h"

#include <move_base_msgs/GetMultiplePlans.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <map>
#include <sstream>
#include <fstream>

#define TURTLEBOT_AGENT_DEBUG

TurtlebotAgent::TurtlebotAgent(tf::TransformListener &tf_listener, const std::string &world_frame_id)
  : m_world_frame_id(world_frame_id)
  , m_tf_listener(tf_listener)
  , m_move_base_client("move_base", true)
{
  ros::NodeHandle nh_priv("~");

  // acquisition_time should include the complete timespan needed to obtain and integrate sensor data once the
  // observation pose has been reached. (in gazebo with sensor_motion_filter use at least 5.0 here)
  if (!nh_priv.getParam("acquisition_time", m_acquisition_time))
  {
    ROS_WARN("Sensor acquisition time not set. Using default!");
    m_acquisition_time = 5.0;
  }
  ROS_INFO_STREAM("Sensor acquisition time: " << m_acquisition_time);

  nh_priv.param("base_frame_id",   m_base_frame_id,   std::string("base_footprint"));
  nh_priv.param("camera_frame_id", m_camera_frame_id, std::string("camera_depth_frame"));

  // resolve TF prefixes
  std::string tf_prefix(tf::getPrefixParam(nh_priv));
  //m_world_frame_id = tf::resolve(tf_prefix, m_world_frame_id);
  m_base_frame_id = tf::resolve(tf_prefix, m_base_frame_id);
  m_camera_frame_id = tf::resolve(tf_prefix, m_camera_frame_id);
}

tf::Pose TurtlebotAgent::getCurrentRobotPose() const
{
  tf::StampedTransform robot_pose;
  m_tf_listener.lookupTransform(m_world_frame_id, m_base_frame_id, ros::Time(), robot_pose);
  return robot_pose;
}

tf::Pose TurtlebotAgent::getCurrentCamPose() const
{
  tf::StampedTransform cam_pose;
  m_tf_listener.lookupTransform(m_world_frame_id, m_camera_frame_id, ros::Time(), cam_pose);
  return cam_pose;
}

/**
  Return the pose of the camera link for a given robot pose.
  */
tf::Pose TurtlebotAgent::camPoseForRobotPose(tf::Pose const &robot_pose) const
{
  tf::StampedTransform base_to_camera;
  m_tf_listener.lookupTransform(m_base_frame_id, m_camera_frame_id, ros::Time(), base_to_camera);
  return robot_pose * base_to_camera;
}

/**
  Return the robot pose for a given camera pose.
  */
tf::Pose TurtlebotAgent::robotPoseForCamPose(tf::Pose const &cam_pose) const
{
  tf::StampedTransform camera_to_base;
  m_tf_listener.lookupTransform(m_camera_frame_id, m_base_frame_id, ros::Time(), camera_to_base);
  return cam_pose * camera_to_base;
}

std::vector<double> TurtlebotAgent::estimateMoveTimes
(
  std::vector<tf::Pose> const &cam_poses,
  std::vector<tf::Pose> const &base_poses,
  std::vector<size_t> const &start_pose_idxs,
  std::vector<size_t> const &target_pose_idxs,
  size_t const n_clusters) const
{
#ifdef TURTLEBOT_AGENT_DEBUG
  // Prepare debug marker publisher
  ros::Publisher dbg_marker_pub(ros::NodeHandle().advertise<visualization_msgs::Marker>("turtlebot_agent_dbg", 10000));
  ros::WallDuration(5.0).sleep();
#endif

  assert(start_pose_idxs.size() == target_pose_idxs.size());
  std::vector<double> times(start_pose_idxs.size(), std::numeric_limits<double>::infinity());

  // Do k-means clustering for all points to minimize actual path planning costs
  std::vector<Kmeans::Point> points(base_poses.size());
  for (size_t i = 0; i < base_poses.size(); ++i)
  {
    points[i].x = base_poses[i].getOrigin().getX();
    points[i].y = base_poses[i].getOrigin().getY();
  }
  Kmeans km(1, points);
  for (size_t i = 1; i < n_clusters; ++i)
  {
    km.addCluster();
  }
  ROS_INFO_STREAM("k-means completed with max_remote_distance=" << km.getMaxRemoteDistance());

#ifdef TURTLEBOT_AGENT_DEBUG
  // Publish cluster centers
  visualization_msgs::Marker clusters_marker;
  clusters_marker.action = visualization_msgs::Marker::ADD;
  clusters_marker.header.frame_id = m_world_frame_id;
  clusters_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  clusters_marker.ns = "cluster_centers";
  clusters_marker.id = 0;
  clusters_marker.color.r = 1.0;
  clusters_marker.color.a = 1.0;
  clusters_marker.scale.x = 0.1;
  clusters_marker.scale.y = 0.1;
  clusters_marker.scale.z = 0.1;
  clusters_marker.points.resize(km.getCentroidIdxs().size());
  for (size_t centroid_idx = 0; centroid_idx < km.getCentroidIdxs().size(); ++centroid_idx)
  {
    Kmeans::Point &p = points[km.getCentroidIdxs()[centroid_idx]];
    clusters_marker.points[centroid_idx].x = p.x;
    clusters_marker.points[centroid_idx].y = p.y;
    clusters_marker.points[centroid_idx].z = 0.0;
  }
  dbg_marker_pub.publish(clusters_marker);
  // Publish assignments
  visualization_msgs::Marker assignment_marker;
  assignment_marker.action = visualization_msgs::Marker::ADD;
  assignment_marker.header.frame_id = m_world_frame_id;
  assignment_marker.type = visualization_msgs::Marker::LINE_LIST;
  assignment_marker.ns = "cluster_assignments";
  assignment_marker.id = 0;
  assignment_marker.color.r = 1.0;
  assignment_marker.color.a = 1.0;
  assignment_marker.scale.x = 0.02;
  assignment_marker.scale.y = 0.02;
  assignment_marker.scale.z = 0.02;
  assignment_marker.points.resize(2 * points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    const Kmeans::Point &p = km.getCentroid(i);
    assignment_marker.points[2 * i].x = p.x;
    assignment_marker.points[2 * i].y = p.y;
    assignment_marker.points[2 * i].z = 0.0;
    assignment_marker.points[2 * i + 1].x = points[i].x;
    assignment_marker.points[2 * i + 1].y = points[i].y;
    assignment_marker.points[2 * i + 1].z = 0.0;
  }
  dbg_marker_pub.publish(assignment_marker);
#endif

  // Compile a map of (origin,target): drive_time between clusters
  typedef std::map<std::pair<size_t, size_t>, double> cluster_pathmap_t;
  cluster_pathmap_t cluster_pathmap;
  for (size_t i = 0; i < start_pose_idxs.size(); ++i)
  {
    size_t origin_cluster = km.getAssignment()[start_pose_idxs[i]];
    size_t target_cluster = km.getAssignment()[target_pose_idxs[i]];
    cluster_pathmap[std::pair<size_t, size_t>(origin_cluster, target_cluster)] =
      -std::numeric_limits<double>::infinity();
  }

  // generate path planner request
  move_base_msgs::GetMultiplePlans get_plans_call;
  get_plans_call.request.start.resize(cluster_pathmap.size());
  get_plans_call.request.goal.resize(cluster_pathmap.size());
  {
    size_t i = 0;
    for (cluster_pathmap_t::iterator it = cluster_pathmap.begin(); it != cluster_pathmap.end(); ++it)
    {
      tf::poseTFToMsg(base_poses[km.getCentroidIdxs()[it->first.first]], get_plans_call.request.start[i].pose);
      get_plans_call.request.start[i].header.frame_id = m_world_frame_id;
      tf::poseTFToMsg(base_poses[km.getCentroidIdxs()[it->first.second]], get_plans_call.request.goal[i].pose);
      get_plans_call.request.goal[i].header = get_plans_call.request.start[i].header;
      ++i;
    }
  }
  // call move_base
  if (!ros::service::call("move_base/make_multiple_plans", get_plans_call))   // TODO: move_base_planner_node?
  {
    ROS_WARN("make_multiple_plans call failed");
    return times;
  }

#ifdef TURTLEBOT_AGENT_DEBUG
  // Publish all paths as markers
  for (size_t i = 0; i < start_pose_idxs.size(); ++i)
  {
    size_t start_pose_idx = start_pose_idxs[i];
    size_t target_pose_idx = target_pose_idxs[i];
    // Figure out the correct connecting path
    size_t origin_cluster = km.getAssignment()[start_pose_idx];
    size_t target_cluster = km.getAssignment()[target_pose_idx];
    size_t path_idx = std::distance(cluster_pathmap.begin(), cluster_pathmap.lower_bound(std::make_pair(origin_cluster, target_cluster)));

    visualization_msgs::Marker path_marker;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.header.frame_id = m_world_frame_id;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.ns = "cluster_paths";
    path_marker.id = i;
    //path_marker.lifetime = ros::Duration(5.0);
    path_marker.color.r = 1.0;
    path_marker.color.g = 1.0;
    path_marker.color.a = 1.0;
    path_marker.scale.x = 0.02;
    path_marker.scale.y = 0.02;
    path_marker.scale.z = 0.02;
    path_marker.points.resize(get_plans_call.response.plans[path_idx].poses.size());
    for (size_t j = 0; j < get_plans_call.response.plans[path_idx].poses.size(); ++j)
    {
      path_marker.points[j].x = get_plans_call.response.plans[path_idx].poses[j].pose.position.x;
      path_marker.points[j].y = get_plans_call.response.plans[path_idx].poses[j].pose.position.y;
      path_marker.points[j].z = get_plans_call.response.plans[path_idx].poses[j].pose.position.z;
    }
    dbg_marker_pub.publish(path_marker);
  }
#endif

  // Fill the cluster_pathmap with drive time estimation for each path
  {
    size_t i = 0;
    for (cluster_pathmap_t::iterator it = cluster_pathmap.begin(); it != cluster_pathmap.end(); ++it)
    {
      if (get_plans_call.response.plans[i].poses.empty())
      {
        it->second = std::numeric_limits<double>::infinity();
      }
      else
      {
        it->second = pathCostToDriveTime(get_plans_call.response.plan_costs[i]);
      }
      ++i;
    }
  }

  for (size_t i = 0; i < times.size(); ++i)
  {
    size_t start_pose_idx = start_pose_idxs[i];
    size_t target_pose_idx = target_pose_idxs[i];

    // Figure out the correct connecting path
    size_t origin_cluster = km.getAssignment()[start_pose_idx];
    size_t target_cluster = km.getAssignment()[target_pose_idx];

    // Set drive time prediction
    times[i] = cluster_pathmap[std::make_pair(origin_cluster, target_cluster)];
  }

  return times;
}

bool TurtlebotAgent::achieveCamPose
(
  tf::Pose const &target_cam_pose,
  double const target_distance)
{
  while (!m_move_base_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  bool success = true;

  const tf::Pose current_base_pose = getCurrentRobotPose();
  const tf::Pose current_cam_pose = getCurrentCamPose();

  // move base if required
  double dh = std::sqrt(std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2) +
                        std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2));
  if (dh > HORIZONTAL_TOLERANCE)
  {
    ROS_INFO("Moving base...");
    tf::Pose target_base_pose = robotPoseForCamPose(target_cam_pose);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = m_world_frame_id;
    tf::poseTFToMsg(target_base_pose, goal.target_pose.pose);

    std::vector<tf::Pose> poses(2);
    poses[0] = current_base_pose;
    poses[1] = target_base_pose;
    double timeout = estimateMoveTimes(std::vector<tf::Pose>(),
                                       poses,
                                       std::vector<size_t>(1, 0),
                                       std::vector<size_t>(1, 1),
                                       2)[0];

    m_move_base_client.sendGoal(goal);
    m_move_base_client.waitForResult(ros::Duration(timeout + 5.0));
    if (m_move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("goal reached with success");
    }
    else
    {
      ROS_INFO("goal approach failed");
      m_move_base_client.cancelAllGoals();
      success = false;
    }
  }

  return success;
}
