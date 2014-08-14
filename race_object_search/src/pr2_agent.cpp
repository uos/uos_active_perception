#include "pr2_agent.h"

#include <nav_msgs/GetPlan.h>

Pr2Agent::Pr2Agent()
:
    m_move_base_client("move_base", true),
    m_point_head_client("/head_traj_controller/point_head_action", true),
    m_lift_torso_client("/torso_controller/position_joint_action", true)
{
    while(!m_move_base_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    };
    while(!m_point_head_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the point_head action server to come up");
    };
    while(!m_lift_torso_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the lift_torso action server to come up");
    };
}


tf::Pose Pr2Agent::cam_pose_for_robot_pose(tf::Pose const & robot_pose) const
{
    tf::Vector3 cam_position = robot_pose.getOrigin();
    cam_position.setZ(CAM_HEIGHT);
    return tf::Pose(robot_pose.getRotation(), cam_position);
}

tf::Pose Pr2Agent::robot_pose_for_cam_pose(tf::Pose const & cam_pose) const
{
    // Calculate the base pose for this camera pose.
    // For pr2 the xy offset between cam and base is assumed to be 0 although it actually depends on cam orientation.
    tf::Vector3 base_position = cam_pose.getOrigin();
    base_position.setZ(0);
    tf::Vector3 xr = tf::Transform(cam_pose.getRotation(), tf::Vector3(0,0,0))(tf::Vector3(1,0,0));
    tf::Quaternion base_rotation;
    base_rotation.setRPY(0, 0, std::atan2(xr.getY(), xr.getX()));
    return tf::Pose(base_rotation, base_position);
}

double Pr2Agent::estimate_move_time
(
        tf::Pose const & current_base_pose,
        tf::Pose const & current_cam_pose,
        tf::Pose const & target_cam_pose,
        std::string const & world_frame_id) const
{
    // initialize exec_time with constant data acquisition time
    double exec_time = ACQUISITION_TIME;

    // calculate the time needed to move_base
    if(is_move_base_required(current_base_pose, target_cam_pose))
    {
        exec_time += estimate_move_base_time(current_base_pose,
                                             robot_pose_for_cam_pose(target_cam_pose),
                                             world_frame_id);
    }

    // calculate time needed to turn the camera
    // TODO: Split into pitch and yaw
    exec_time += current_cam_pose.getRotation().angleShortestPath(target_cam_pose.getRotation()) / HEAD_SPEED;

    // calculate time needed to lift the torso
    double dz = std::abs(current_cam_pose.getOrigin().getZ() - target_cam_pose.getOrigin().getZ());
    if(dz > TRANSLATIONAL_TOLERANCE)
    {
        exec_time += dz / LIFT_SPEED;
    }

    return exec_time;
}

bool Pr2Agent::is_move_base_required
(
        tf::Pose const & robot_pose,
        tf::Pose const & target_cam_pose) const
{
    tf::Pose cam_pose = cam_pose_for_robot_pose(robot_pose);
    // TODO: Check extreme camera yaw (needs base rotation)
    double distance = std::sqrt(std::pow(cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2) +
                                std::pow(cam_pose.getOrigin().getY() - target_cam_pose.getOrigin().getY(), 2));
    return distance > TRANSLATIONAL_TOLERANCE;
}

double Pr2Agent::estimate_move_base_time
(
        tf::Pose const & current_base_pose,
        tf::Pose const & target_base_pose,
        std::string const & frame_id) const
{
    if(is_move_base_required(current_base_pose, cam_pose_for_robot_pose(target_base_pose)))
    {
        // generate path planner request
        nav_msgs::GetPlan get_plan_call;
        tf::poseTFToMsg(current_base_pose, get_plan_call.request.start.pose);
        get_plan_call.request.start.header.frame_id = frame_id;
        get_plan_call.request.start.header.stamp = ros::Time::now();
        tf::poseTFToMsg(target_base_pose, get_plan_call.request.goal.pose);
        get_plan_call.request.goal.header = get_plan_call.request.start.header;
        get_plan_call.request.tolerance = TRANSLATIONAL_TOLERANCE;
        if(!ros::service::call("/move_base_node/make_plan", get_plan_call))
        {
            ROS_WARN("make_plan call failed");
            return std::numeric_limits<double>::infinity();
        }
        std::vector<geometry_msgs::PoseStamped> const & path = get_plan_call.response.plan.poses;
        if(path.empty())
        {
            // no plan found
            return std::numeric_limits<double>::infinity();
        }
        double path_len = 0, path_curvature = 0;
        for(unsigned int i_path = 1; i_path < path.size(); i_path++)
        {
            tf::Pose p1, p2;
            tf::poseMsgToTF(path[i_path - 1].pose, p1);
            tf::poseMsgToTF(path[i_path].pose, p2);
            path_len += p1.getOrigin().distance(p2.getOrigin());
            path_curvature += p1.getRotation().angleShortestPath(p2.getRotation());
        }
        return path_len / DRIVE_SPEED + path_curvature / TURN_SPEED;
    }
    else
    {
        return 0;
    }
}

bool Pr2Agent::achieve_cam_pose
(
        tf::Pose const & current_base_pose,
        tf::Pose const & current_cam_pose,
        tf::Pose const & target_cam_pose,
        std::string const & world_frame_id)
{
    // move base if required
    if(is_move_base_required(current_base_pose, target_cam_pose))
    {
        tf::Pose target_base_pose = robot_pose_for_cam_pose(target_cam_pose);
        ROS_INFO("Moving base...");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = world_frame_id;
        goal.target_pose.header.stamp = ros::Time::now();
        tf::poseTFToMsg(target_base_pose, goal.target_pose.pose);

        double timeout = estimate_move_base_time(current_base_pose, target_base_pose, world_frame_id);
        if(timeout > 9000)
        {
            return false;
        }
        m_move_base_client.sendGoal(goal);
        m_move_base_client.waitForResult(ros::Duration(timeout + 5.0));
        if(m_move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("goal reached with success");
        } else {
            ROS_INFO("goal approach failed");
            m_move_base_client.cancelAllGoals();
            return false;
        }
    }

    // lift torso if required
    if(std::abs(current_cam_pose.getOrigin().getZ() - target_cam_pose.getOrigin().getZ()) > TRANSLATIONAL_TOLERANCE)
    {
        ROS_INFO("Lifting torso...");
        pr2_controllers_msgs::SingleJointPositionGoal torso_goal;
        torso_goal.position = torso_position_for_cam_height(target_cam_pose.getOrigin().getZ());
        m_lift_torso_client.sendGoal(torso_goal);
        m_lift_torso_client.waitForResult(ros::Duration(7.0));
        if(m_lift_torso_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("torso lifted with success");
        } else {
            ROS_INFO("torso failed");
            m_lift_torso_client.cancelAllGoals();
            return false;
        }
    }

    ROS_INFO("Pointing head...");
    pr2_controllers_msgs::PointHeadGoal head_goal;
    head_goal.target.header.frame_id = world_frame_id;
    head_goal.target.header.stamp = ros::Time::now();
    tf::pointTFToMsg(target_cam_pose * tf::Point(1e10,0,0), head_goal.target.point);

    m_point_head_client.sendGoal(head_goal);
    m_point_head_client.waitForResult(ros::Duration(3.0));
    if(m_point_head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("head moved with success");
    } else {
        ROS_INFO("head movement failed");
        m_point_head_client.cancelAllGoals();
        return false;
    }

    return true;
}
