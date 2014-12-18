#ifndef FLOATING_KINECT_AGENT_H
#define FLOATING_KINECT_AGENT_H

#include "pr2_agent.h"

#include <string>

/**
  Delegates everything to PR2 Agent except for achieveCamPose which directly sets the camera pose in Gazebo.
  */
class FloatingKinectAgent
{
public:
    FloatingKinectAgent(tf::TransformListener & tf_listener, const std::string & world_frame_id);

    double getAcquisitionTime() const;

    tf::Pose getCurrentRobotPose() const;

    tf::Pose getCurrentCamPose() const;

    tf::Pose camPoseForRobotPose(tf::Pose const & robot_pose) const;

    tf::Pose robotPoseForCamPose(tf::Pose const & cam_pose) const;

    std::vector<double> estimateMoveTimes(std::vector<tf::Pose> const & cam_poses,
                                          std::vector<tf::Pose> const & base_poses,
                                          std::vector<size_t> const & start_pose_idxs,
                                          std::vector<size_t> const & target_pose_idxs,
                                          size_t const n_clusters) const;

    bool achieveCamPose(tf::Pose const & target_cam_pose, double const target_distance);

private: 
    const tf::Transform sim_map_tf;
    Pr2Agent m_pr2;
};

#endif // FLOATING_KINECT_AGENT_H
