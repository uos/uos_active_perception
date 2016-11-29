#ifndef TURTLEBOT_AGENT_H
#define TURTLEBOT_AGENT_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

/**
  For PR2, the camera link is simplified to allow for panning and tilting of the camera without changing the position
  relative to /base_footprint. This virtual camera link is located 0.292 m above /head_pan_link.
  */
class TurtlebotAgent
{
public:
  TurtlebotAgent(tf::TransformListener &tf_listener, const std::string &world_frame_id);

  double getAcquisitionTime() const
  {
    return m_acquisition_time;
  }

  tf::Pose getCurrentRobotPose() const;

  tf::Pose getCurrentCamPose() const;

  tf::Pose camPoseForRobotPose(tf::Pose const &robot_pose) const;

  tf::Pose robotPoseForCamPose(tf::Pose const &cam_pose) const;

  std::vector<double> estimateMoveTimes(std::vector<tf::Pose> const &cam_poses,
                                        std::vector<tf::Pose> const &base_poses,
                                        std::vector<size_t> const &start_pose_idxs,
                                        std::vector<size_t> const &target_pose_idxs,
                                        size_t const n_clusters) const;

  bool achieveCamPose(tf::Pose const & target_cam_pose, double const target_distance);

private:
  // execution speed constants
  double static const DRIVE_TIME_TOLERANCE = 8; // [s]
  double static const TURN_SPEED = 1.57079632679; // [rad/s]
  double static const HORIZONTAL_TOLERANCE = 0.05;
  double static const VERTICAL_TOLERANCE = 0.05;

  /**
    Drive time prediction based on linear regression.
    */
  double static pathCostToDriveTime(const double path_cost)
  {
    return 0.000305329882947 * path_cost + 2.72422335968;
  }

  double m_acquisition_time;
  const std::string &m_world_frame_id;
  std::string m_base_frame_id, m_camera_frame_id;
  tf::TransformListener &m_tf_listener;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_move_base_client;
};

#endif // TURTLEBOT_AGENT_H
