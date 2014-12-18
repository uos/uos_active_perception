#include "floating_kinect_agent.h"

#include <gazebo_msgs/SetModelState.h>

#include <map>
#include <sstream>
#include <fstream>

FloatingKinectAgent::FloatingKinectAgent(tf::TransformListener & tf_listener, const std::string & world_frame_id)
:
    sim_map_tf(tf::Quaternion::getIdentity(), tf::Point(-12.28, -10.20, 0.0)),
    m_pr2(tf_listener, world_frame_id)
{}

double FloatingKinectAgent::getAcquisitionTime() const
{
    return m_pr2.getAcquisitionTime();
}

tf::Pose FloatingKinectAgent::getCurrentRobotPose() const
{
    return m_pr2.getCurrentRobotPose();
}

tf::Pose FloatingKinectAgent::getCurrentCamPose() const
{
    return m_pr2.getCurrentCamPose();
}

tf::Pose FloatingKinectAgent::camPoseForRobotPose(tf::Pose const & robot_pose) const
{
    return m_pr2.camPoseForRobotPose(robot_pose);
}

tf::Pose FloatingKinectAgent::robotPoseForCamPose(tf::Pose const & cam_pose) const
{
    return m_pr2.robotPoseForCamPose(cam_pose);
}

std::vector<double> FloatingKinectAgent::estimateMoveTimes
(
        std::vector<tf::Pose> const & cam_poses,
        std::vector<tf::Pose> const & base_poses,
        std::vector<size_t> const & start_pose_idxs,
        std::vector<size_t> const & target_pose_idxs,
        size_t const n_clusters) const
{
    return m_pr2.estimateMoveTimes(cam_poses, base_poses, start_pose_idxs, target_pose_idxs, n_clusters);
}

bool FloatingKinectAgent::achieveCamPose
(
        tf::Pose const & target_cam_pose,
        double const target_distance)
{ 
    gazebo_msgs::SetModelState set_model_state;
    set_model_state.request.model_state.model_name = "floating_kinect";
    tf::poseTFToMsg(sim_map_tf * target_cam_pose, set_model_state.request.model_state.pose);
    ros::service::call("/gazebo/set_model_state", set_model_state);
    return true;
}
