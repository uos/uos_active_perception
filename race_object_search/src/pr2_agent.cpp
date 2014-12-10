#include "pr2_agent.h"

#include "kmeans.h"

#include <move_base/GetMultiplePlans.h>
#include <nav_msgs/Path.h>

#include <map>
#include <sstream>
#include <fstream>

double Pr2Agent::torsoPositionForCamHeight(double const & cam_height)
{
    // torso high: position=0.32, head_mount_kinect_ir_link z=1.784
    // torso low : position=0.02, head_mount_kinect_ir_link z=1.484
    double torso_state = cam_height - 1.464;
    if(torso_state > 0.32) {
        return 0.32;
    }
    else if(torso_state < 0.02) {
        return 0.02;
    }
    else {
        return torso_state;
    }
}

Pr2Agent::Pr2Agent(tf::TransformListener & tf_listener, const std::string & world_frame_id)
:
    m_world_frame_id(world_frame_id),
    m_tf_listener(tf_listener),
    m_move_base_client("move_base", true),
    m_point_head_client("/point_head_continuous/point_head_action", true),
    m_lift_torso_client("/torso_controller/position_joint_action", true)
{
    // acquisition_time should include the complete timespan needed to obtain and integrate sensor data once the
    // observation pose has been reached. (in gazebo with sensor_motion_filter use at least 8.0 here)
    ros::param::param<double>("~acquisition_time", ACQUISITION_TIME, 1.0);

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

tf::Pose Pr2Agent::getCurrentRobotPose() const
{
    tf::StampedTransform robot_pose;
    m_tf_listener.lookupTransform(m_world_frame_id, "/base_footprint", ros::Time(), robot_pose);
    return robot_pose;
}

tf::Pose Pr2Agent::getCurrentCamPose() const
{
    tf::StampedTransform cam_pose;
    m_tf_listener.lookupTransform(m_world_frame_id, "/head_pan_link", ros::Time(), cam_pose);
    cam_pose.getOrigin() += tf::Vector3(0.0, 0.0, 0.292);
    return cam_pose;
}

/**
  Return the pose of the virtual (fixed) camera link for a given robot pose with the current torso height.
  View direction is set to match camera pan but not tilt.
  */
tf::Pose Pr2Agent::camPoseForRobotPose(tf::Pose const & robot_pose) const
{
    try {
        tf::StampedTransform head_pan_link_tf;
        m_tf_listener.lookupTransform("/base_footprint", "/head_pan_link", ros::Time(), head_pan_link_tf);
        return robot_pose * (head_pan_link_tf * tf::Pose(tf::Quaternion::getIdentity(), tf::Point(0.0, 0.0, 0.292)));
    } catch(tf::TransformException& ex) {
        ROS_ERROR_STREAM("Pr2Agent: Unable to lookup transform: "
                         << ex.what()
                         << ", using minimum camera height instead.");
        return robot_pose * tf::Pose(tf::Quaternion::getIdentity(), tf::Point(-0.07, 0.0, 1.484));
    }
}

/**
  Return a robot pose for a given virtual (fixed) camera link position that is oriented towards the camera view
  direction.
  */
tf::Pose Pr2Agent::robotPoseForCamPose(tf::Pose const & cam_pose) const
{
    tf::Vector3 projected_position = cam_pose.getOrigin();
    projected_position.setZ(0);
    tf::Vector3 xr = tf::Transform(cam_pose.getRotation(), tf::Vector3(0,0,0))(tf::Vector3(1,0,0));
    tf::Quaternion base_rotation;
    base_rotation.setRPY(0, 0, std::atan2(xr.getY(), xr.getX()));
    tf::Pose pan_link_projected(base_rotation, projected_position);
    tf::Pose base_offset(tf::Quaternion::getIdentity(), tf::Point(0.07, 0.0, 0.0));
    return pan_link_projected * base_offset;
}

std::vector<double> Pr2Agent::estimateMoveTimes
(
        std::vector<tf::Pose> const & cam_poses,
        std::vector<tf::Pose> const & base_poses,
        std::vector<size_t> const & start_pose_idxs,
        std::vector<size_t> const & target_pose_idxs,
        size_t const n_clusters) const
{
    // estimate move_base times
    std::vector<double> times = estimateMoveBaseTimes(base_poses,
                                                         start_pose_idxs,
                                                         target_pose_idxs,
                                                         n_clusters);

    for(size_t i = 0; i < times.size(); ++i) {
        // calculate time needed to lift the torso
        double dz = std::abs(cam_poses[start_pose_idxs[i]].getOrigin().getZ() -
                             cam_poses[target_pose_idxs[i]].getOrigin().getZ());
        if(dz > VERTICAL_TOLERANCE)
        {
            times[i] += dz / LIFT_SPEED;
        }

        // Calculate time needed to turn the camera if nothing else needs to move.
        // Otherwise, head movement is parallelized and does not cost extra time.
        if(times[i] < std::numeric_limits<double>::epsilon()) {
            times[i] += cam_poses[start_pose_idxs[i]].getRotation().angleShortestPath(
                            cam_poses[target_pose_idxs[i]].getRotation()) / HEAD_SPEED;
        }

        // constant data acquisition time
        times[i] += ACQUISITION_TIME;
    }

    return times;
}

std::vector<double> Pr2Agent::estimateMoveBaseTimes
(
        std::vector<tf::Pose> const & poses,
        std::vector<size_t> const & start_pose_idxs,
        std::vector<size_t> const & target_pose_idxs,
        size_t const n_clusters) const
{  
    assert(start_pose_idxs.size() == target_pose_idxs.size());
    std::vector<double> times(start_pose_idxs.size(), std::numeric_limits<double>::infinity());

    // Do k-means clustering for all points to minimize actual path planning costs
    std::vector<Kmeans::Point> points(poses.size());
    for(size_t i = 0; i < poses.size(); ++i) {
        points[i].x = poses[i].getOrigin().getX();
        points[i].y = poses[i].getOrigin().getY();
    }
    Kmeans km(1, points);
    for(size_t i = 1; i < n_clusters; ++i) {
        km.addCluster();
    }
    ROS_INFO_STREAM("k-means completed with max_remote_distance=" << km.getMaxRemoteDistance());
    // Uncomment to dump clustering result
    // km.writeTabFiles();

    // Compile a map of (origin,target):path between clusters
    typedef std::map<std::pair<size_t, size_t>, std::pair<double, double> > cluster_pathmap_t;
    cluster_pathmap_t cluster_pathmap;
    for(size_t i = 0; i < start_pose_idxs.size(); ++i) {
        size_t origin_cluster = km.getAssignment()[start_pose_idxs[i]];
        size_t target_cluster = km.getAssignment()[target_pose_idxs[i]];
        cluster_pathmap[std::pair<size_t, size_t>(origin_cluster, target_cluster)] =
                std::make_pair(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
    }

    // generate path planner request
    move_base::GetMultiplePlans get_plans_call;
    get_plans_call.request.start.resize(cluster_pathmap.size());
    get_plans_call.request.goal.resize(cluster_pathmap.size());
    {
        size_t i = 0;
        for(cluster_pathmap_t::iterator it = cluster_pathmap.begin(); it != cluster_pathmap.end(); ++it) {
            tf::poseTFToMsg(poses[km.getCentroidIdxs()[it->first.first]], get_plans_call.request.start[i].pose);
            get_plans_call.request.start[i].header.frame_id = m_world_frame_id;
            get_plans_call.request.start[i].header.stamp = ros::Time::now();
            tf::poseTFToMsg(poses[km.getCentroidIdxs()[it->first.second]], get_plans_call.request.goal[i].pose);
            get_plans_call.request.goal[i].header = get_plans_call.request.start[i].header;
            ++i;
        }
    }
    // call move_base
    if(!ros::service::call("/move_base_node/make_multiple_plans", get_plans_call)) {
        ROS_WARN("make_multiple_plans call failed");
        return times;
    }

    // Uncomment to dump all generated paths to tab files
    /*
    for(size_t i = 0; i < get_plans_call.response.plans.size(); ++i) {
        std::stringstream ss;
        ss << "path_" << i << ".tab";
        std::ofstream f;
        f.open(ss.str().c_str());
        f << "x\ty\ttype\n";
        f << "c\tc\td\n";
        f << "\t\tc\n";
        f << get_plans_call.request.start[i].pose.position.x << "\t"
          << get_plans_call.request.start[i].pose.position.y << "\t" << "start" << "\n";
        for(size_t j = 0; j < get_plans_call.response.plans[i].poses.size(); ++j) {
            f << get_plans_call.response.plans[i].poses[j].pose.position.x << "\t"
              << get_plans_call.response.plans[i].poses[j].pose.position.y << "\t" << "path" << "\n";
        }
        f << get_plans_call.request.goal[i].pose.position.x << "\t"
          << get_plans_call.request.goal[i].pose.position.y << "\t" << "goal" << "\n";
        f.close();
        ss.str("");
        ss << "path_" << i << ".txt";
        f.open(ss.str().c_str());
        f << get_plans_call.response.plans[i];
        f.close();
    }
    */

    // Fill the cluster_pathmap with (length, curvature) of each path
    {
        size_t i = 0;
        for(cluster_pathmap_t::iterator it = cluster_pathmap.begin(); it != cluster_pathmap.end(); ++it) {
            double length = 0, curvature = 0;
            std::vector<geometry_msgs::PoseStamped> const & p = get_plans_call.response.plans[i].poses;
            if(p.empty()) {
                length = std::numeric_limits<double>::infinity();
            } else {
                double x1 = p[0].pose.position.x;
                double y1 = p[0].pose.position.y;
                for(size_t i_path = 1; i_path + 1 < p.size(); i_path++) {
                    double x2 = p[i_path].pose.position.x;
                    double y2 = p[i_path].pose.position.y;
                    length += std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
                    // TODO: calculate curvature
                    x1 = x2;
                    y1 = y2;
                }
            }
            it->second = std::make_pair(length, curvature);
            ++i;
        }
    }

    for(size_t i = 0; i < times.size(); ++i) {
        size_t start_pose_idx = start_pose_idxs[i];
        size_t target_pose_idx = target_pose_idxs[i];
        // Figure out the correct connecting path
        size_t origin_cluster = km.getAssignment()[start_pose_idx];
        size_t target_cluster = km.getAssignment()[target_pose_idx];
        std::pair<double, double> const & path_length_curvature =
                cluster_pathmap[std::make_pair(origin_cluster, target_cluster)];
        // Estimate time
        times[i] = path_length_curvature.first / DRIVE_SPEED
                   + path_length_curvature.second / TURN_SPEED
                   + poses[start_pose_idxs[i]].getRotation().angleShortestPath(
                     poses[target_pose_idxs[i]].getRotation()) / TURN_SPEED;
    }

    return times;
}

bool Pr2Agent::achieveCamPose
(
        tf::Pose const & target_cam_pose,
        double const target_distance)
{ 
    bool success = true;

    const tf::Pose current_base_pose = getCurrentRobotPose();
    const tf::Pose current_cam_pose = getCurrentCamPose();

    // lock head onto target
    pr2_controllers_msgs::PointHeadGoal head_goal;
    head_goal.target.header.frame_id = m_world_frame_id;
    head_goal.target.header.stamp = ros::Time::now();
    head_goal.pointing_frame = "/head_mount_kinect_ir_link";
    head_goal.pointing_axis.x = 1;
    head_goal.pointing_axis.y = 0;
    head_goal.pointing_axis.z = 0;
    tf::pointTFToMsg(target_cam_pose * tf::Point(target_distance, 0.0, 0.0), head_goal.target.point);
    m_point_head_client.sendGoal(head_goal);
    ROS_INFO_STREAM("looking towards target: stamp=" << head_goal.target.header.stamp);

    // move base if required
    double dh = std::sqrt(std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2) +
                          std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2));
    if(dh > HORIZONTAL_TOLERANCE)
    {
        ROS_INFO("Moving base...");
        tf::Pose target_base_pose = robotPoseForCamPose(target_cam_pose);
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = m_world_frame_id;
        //goal.target_pose.header.stamp = ros::Time::now();
        tf::poseTFToMsg(target_base_pose, goal.target_pose.pose);

        std::vector<tf::Pose> poses(2);
        poses[0] = current_base_pose;
        poses[1] = target_base_pose;
        double timeout = estimateMoveBaseTimes(poses,
                                                  std::vector<size_t>(1, 0),
                                                  std::vector<size_t>(1, 1),
                                                  2)[0];

        m_move_base_client.sendGoal(goal);
        m_move_base_client.waitForResult(ros::Duration(timeout + 5.0));
        if(m_move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("goal reached with success");
        } else {
            ROS_INFO("goal approach failed");
            m_move_base_client.cancelAllGoals();
            success = false;
        }
    }

    // lift torso if required
    double dz = std::abs(current_cam_pose.getOrigin().getZ() - target_cam_pose.getOrigin().getZ());
    if(dz > VERTICAL_TOLERANCE)
    {
        ROS_INFO("Lifting torso...");
        pr2_controllers_msgs::SingleJointPositionGoal torso_goal;
        torso_goal.position = torsoPositionForCamHeight(target_cam_pose.getOrigin().getZ());
        m_lift_torso_client.sendGoal(torso_goal);
        m_lift_torso_client.waitForResult(ros::Duration(dz / LIFT_SPEED + 2.0));
        if(m_lift_torso_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("torso lifted with success");
        } else {
            ROS_INFO("torso failed");
            m_lift_torso_client.cancelAllGoals();
            success = false;
        }
    }

    // Unlock head
    m_point_head_client.cancelGoal();

    return success;
}
