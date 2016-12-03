#include "observation_pose_collection.h"
#include "turtlebot_agent.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <uos_active_perception_msgs/GetObservationCameraPoses.h>

class NbvReachabilityFilterNode
{
public:
  NbvReachabilityFilterNode()
    : m_node_handle("~")
    , m_node_handle_pub()
    , m_tfl()
    , m_agent(m_tfl, m_world_frame_id)
    , m_get_observation_camera_poses_server(m_node_handle_pub.advertiseService(
        "get_observation_camera_poses_reachable",
        &NbvReachabilityFilterNode::getObservationCameraPosesCb,
        this))
  {
    // m_world_frame_id is read here *after* passing it to m_agent. This *only* works
    // because m_agent reads it as a reference and doesn't use it in the constructor,
    // so by the time it is used by m_agent it is correctly set. This is dangerous!
    m_node_handle.param("world_frame_id", m_world_frame_id, std::string("odom_combined"));
    ROS_INFO("%s initialized.", ros::this_node::getName().c_str());
  }

private:
  ros::NodeHandle m_node_handle, m_node_handle_pub;
  tf::TransformListener m_tfl;
  std::string m_world_frame_id;
  TurtlebotAgent m_agent;
  ros::ServiceServer m_get_observation_camera_poses_server;

  bool getObservationCameraPosesCb(uos_active_perception_msgs::GetObservationCameraPoses::Request& req,
                                   uos_active_perception_msgs::GetObservationCameraPoses::Response& res)
  {
    // get samples
    uos_active_perception_msgs::GetObservationCameraPoses pose_candidates_call;
    pose_candidates_call.request = req;

    if (req.omit_cvm)
    {
      // The request field `omit_cm` was set to true. Setting to false because
      // ObservationPoseCollection requires it.
      pose_candidates_call.request.omit_cvm = false;
    }

    if (!ros::service::call("get_observation_camera_poses", pose_candidates_call))
    {
      ROS_ERROR("Could not call get_observation_camera_poses service!");
      return false;
    }

    res.object_sets = pose_candidates_call.response.object_sets;
    res.roi_cell_counts = pose_candidates_call.response.roi_cell_counts;

    ObservationPoseCollection opc;
    opc.addPoses(pose_candidates_call.response.camera_poses,
                 pose_candidates_call.response.target_points,
                 pose_candidates_call.response.cvms,
                 pose_candidates_call.response.object_sets);

    // calc travel times
    opc.prepareInitialTravelTimeLut(m_agent,
                                    m_agent.getCurrentRobotPose(),
                                    m_agent.getCurrentCamPose(),
                                    m_world_frame_id);

    size_t num_poses(opc.getPoses().size());
    if (pose_candidates_call.response.camera_poses.size()      != num_poses ||
        pose_candidates_call.response.target_points.size()     != num_poses ||
        pose_candidates_call.response.information_gains.size() != num_poses ||
        !(pose_candidates_call.response.cvms.size() == num_poses ||
          pose_candidates_call.response.cvms.size() == 0))
    {
      ROS_ERROR("The wrapped service returned an invalid result!");
      return false;
    }

    // add reachable poses to response
    res.camera_poses.reserve(num_poses);
    res.target_points.reserve(num_poses);
    res.information_gains.reserve(num_poses);
    res.cvms.reserve(num_poses);

    for (size_t k = 0; k < num_poses; ++k)
    {
      if (opc.getInitialTravelTime(k) < std::numeric_limits<double>::infinity())
      {
        res.target_points.push_back(pose_candidates_call.response.target_points[k]);
        res.information_gains.push_back(pose_candidates_call.response.information_gains[k]);
        if (pose_candidates_call.response.cvms.size() >= k)
          res.cvms.push_back(pose_candidates_call.response.cvms[k]);

        // HACK: Pass on the base_footprint instead of the camera poses
        geometry_msgs::Pose base_pose;
        tf::poseTFToMsg(m_agent.robotPoseForCamPose(opc.getPoses()[k].pose), base_pose);
        res.camera_poses.push_back(base_pose);
      }
    }

    if (req.omit_cvm)
      res.cvms.clear();

    return true;
  }
};

int main(int argc, char** argv)
{
  std::string nname("nbv_reachability_filter_node");
  ros::init(argc, argv, nname);
  NbvReachabilityFilterNode node;
  ros::spin();
}
