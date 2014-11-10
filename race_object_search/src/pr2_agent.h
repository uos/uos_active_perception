#ifndef PR2_AGENT_H
#define PR2_AGENT_H

#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

class Pr2Agent
{
public:
    Pr2Agent();

    double get_acquisition_time() const { return ACQUISITION_TIME; }

    std::string getRobotPoseFrameId() const { return "/base_footprint"; }

    std::string getRobotCameraFrameId() const { return "/head_mount_kinect_ir_link"; }

    double torso_position_for_cam_height(double const & cam_height) const
    {
        double torso_state = 1.64410058027 * cam_height - 2.41576402321;
        if(torso_state > 0.2) {
            return 0.2;
        }
        else if(torso_state < 0.03) {
            return 0.03;
        }
        else {
            return torso_state;
        }
    }

    tf::Pose cam_pose_for_robot_pose(tf::Pose const & robot_pose) const;

    tf::Pose robot_pose_for_cam_pose(tf::Pose const & cam_pose) const;

    std::vector<double> estimate_move_times(std::vector<tf::Pose> const & cam_poses,
                                            std::vector<tf::Pose> const & base_poses,
                                            std::vector<size_t> const & start_pose_idxs,
                                            std::vector<size_t> const & target_pose_idxs,
                                            size_t const n_clusters,
                                            std::string const & world_frame_id) const;

    bool is_move_base_required(tf::Pose const & robot_pose,
                               tf::Pose const & target_cam_pose) const;

    std::vector<double> estimate_move_base_times(std::vector<tf::Pose> const & poses,
                                                 std::vector<size_t> const & start_pose_idxs,
                                                 std::vector<size_t> const & target_pose_idxs,
                                                 size_t const n_clusters,
                                                 std::string const & frame_id) const;

    bool achieve_cam_pose(tf::Pose const & current_base_pose,
                          tf::Pose const & current_cam_pose,
                          tf::Pose const & target_cam_pose,
                          std::string const & world_frame_id);

private:
    double static const CAM_HEIGHT = 1.5;

    // execution speed constants
    double static const DRIVE_SPEED = 0.25; // [m/s]
    double static const TURN_SPEED = 0.5; // [rad/s]
    double static const LIFT_SPEED = 0.02; // [m/s]
    double static const HEAD_SPEED = 3.14; // [rad/s]
    double static const TRANSLATIONAL_TOLERANCE = 0.1;

    // read as ros parameter
    double ACQUISITION_TIME;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_move_base_client;
    actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> m_point_head_client;
    actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> m_lift_torso_client;
};

#endif // PR2_AGENT_H
