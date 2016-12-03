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

    if (!ros::service::call("get_observation_camera_poses", pose_candidates_call))
    {
      ROS_ERROR("Could not call get_observation_camera_poses service!");
      return false;
    }

    res.object_sets = pose_candidates_call.response.object_sets;

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

    // add reachable poses to response
    res.roi_cell_counts.reserve(opc.getPoses().size());
    res.camera_poses.reserve(opc.getPoses().size());
    res.target_points.reserve(opc.getPoses().size());
    res.information_gains.reserve(opc.getPoses().size());
    res.cvms.reserve(opc.getPoses().size());

    for (size_t k = 0; k < opc.getPoses().size(); ++k)
    {
      if (opc.getInitialTravelTime(k) < std::numeric_limits<double>::infinity())
      {
        res.roi_cell_counts.push_back(pose_candidates_call.response.roi_cell_counts[k]);
        res.target_points.push_back(pose_candidates_call.response.target_points[k]);
        res.information_gains.push_back(pose_candidates_call.response.information_gains[k]);
        res.cvms.push_back(pose_candidates_call.response.cvms[k]);

        // HACK: Pass on the base_footprint instead of the camera poses
        geometry_msgs::Pose base_pose;
        tf::poseTFToMsg(m_agent.robotPoseForCamPose(opc.getPoses()[k].pose), base_pose);
        res.camera_poses.push_back(base_pose);
      }
    }

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
