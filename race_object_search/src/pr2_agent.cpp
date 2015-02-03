#include "pr2_agent.h"

#include "kmeans.h"

#include <move_base/GetMultiplePlans.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

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
    m_move_base_client("/move_base", true),
    m_point_head_client("/point_head_continuous/point_head_action", true),
    m_lift_torso_client("/torso_controller/position_joint_action", true)
{
    // acquisition_time should include the complete timespan needed to obtain and integrate sensor data once the
    // observation pose has been reached. (in gazebo with sensor_motion_filter use at least 8.0 here)
    ros::param::param<double>("~acquisition_time", ACQUISITION_TIME, 1.0);
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
    try {
        m_tf_listener.lookupTransform(m_world_frame_id, "/head_pan_link", ros::Time(), cam_pose);
        cam_pose.getOrigin() += tf::Vector3(0.0, 0.0, 0.292);
        return cam_pose;
    } catch(tf::TransformException& ex) {
        return camPoseForRobotPose(getCurrentRobotPose());
    }
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
    #ifdef PR2_AGENT_DEBUG
    // Prepare debug marker publisher
    ros::Publisher dbg_marker_pub(ros::NodeHandle().advertise<visualization_msgs::Marker>("/pr2_agent_dbg", 10000));
    ros::WallDuration(5.0).sleep();
    #endif

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

    #ifdef PR2_AGENT_DEBUG
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
    for(size_t centroid_idx = 0; centroid_idx < km.getCentroidIdxs().size(); ++centroid_idx)
    {
        Kmeans::Point & p = points[km.getCentroidIdxs()[centroid_idx]];
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
    for(size_t i = 0; i < points.size(); ++i)
    {
        const Kmeans::Point & p = km.getCentroid(i);
        assignment_marker.points[2*i].x = p.x;
        assignment_marker.points[2*i].y = p.y;
        assignment_marker.points[2*i].z = 0.0;
        assignment_marker.points[2*i+1].x = points[i].x;
        assignment_marker.points[2*i+1].y = points[i].y;
        assignment_marker.points[2*i+1].z = 0.0;

    }
    dbg_marker_pub.publish(assignment_marker);
    #endif

    // Compile a map of (origin,target):drive_time between clusters
    typedef std::map<std::pair<size_t, size_t>, double> cluster_pathmap_t;
    cluster_pathmap_t cluster_pathmap;
    for(size_t i = 0; i < start_pose_idxs.size(); ++i) {
        size_t origin_cluster = km.getAssignment()[start_pose_idxs[i]];
        size_t target_cluster = km.getAssignment()[target_pose_idxs[i]];
        cluster_pathmap[std::pair<size_t, size_t>(origin_cluster, target_cluster)] =
                -std::numeric_limits<double>::infinity();
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
            tf::poseTFToMsg(poses[km.getCentroidIdxs()[it->first.second]], get_plans_call.request.goal[i].pose);
            get_plans_call.request.goal[i].header = get_plans_call.request.start[i].header;
            ++i;
        }
    }
    // call move_base
    if(!ros::service::call("/move_base_planner_node/make_multiple_plans", get_plans_call)) {
        ROS_WARN("make_multiple_plans call failed");
        return times;
    }

    #ifdef PR2_AGENT_DEBUG
    // Publish all paths as markers
    dbg_path_markers.clear();
    for(size_t i = 0; i < start_pose_idxs.size(); ++i) {
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
        for(size_t j = 0; j < get_plans_call.response.plans[path_idx].poses.size(); ++j) {
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
        for(cluster_pathmap_t::iterator it = cluster_pathmap.begin(); it != cluster_pathmap.end(); ++it) {
            if(get_plans_call.response.plans[i].poses.empty()) {
                it->second = std::numeric_limits<double>::infinity();
            } else {
                it->second = get_plans_call.response.plan_costs[i];
            }
            ++i;
        }
    }

    for(size_t i = 0; i < times.size(); ++i) {
        size_t start_pose_idx = start_pose_idxs[i];
        size_t target_pose_idx = target_pose_idxs[i];
        // Figure out the correct connecting path
        size_t origin_cluster = km.getAssignment()[start_pose_idx];
        size_t target_cluster = km.getAssignment()[target_pose_idx];
        // Set drive time prediction
        double yaw_diff = poses[start_pose_idxs[i]].getRotation().angleShortestPath(
                            poses[target_pose_idxs[i]].getRotation());
        assert(yaw_diff >= 0);
        times[i] = cluster_pathmap[std::make_pair(origin_cluster, target_cluster)]
                   + yaw_diff / TURN_SPEED;
    }

    return times;
}

bool Pr2Agent::achieveCamPose
(
        tf::Pose const & target_cam_pose,
        double const target_distance)
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

    bool success = true;

    const tf::Pose current_base_pose = getCurrentRobotPose();
    const tf::Pose current_cam_pose = getCurrentCamPose();

    // lock head onto target
    pr2_controllers_msgs::PointHeadGoal head_goal;
    head_goal.target.header.frame_id = m_world_frame_id;
    head_goal.target.header.stamp = ros::Time::now();
    head_goal.pointing_frame = "/head_mount_kinect_rgb_link";
    head_goal.pointing_axis.x = 1;
    head_goal.pointing_axis.y = 0;
    head_goal.pointing_axis.z = 0;
    tf::pointTFToMsg(target_cam_pose * tf::Point(target_distance, 0.0, 0.0), head_goal.target.point);
    m_point_head_client.sendGoal(head_goal);
    ROS_INFO_STREAM("looking towards target: stamp=" << head_goal.target.header.stamp);

    // move base if required
    bool base_moved = false;
    double dh = std::sqrt(std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2) +
                          std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2));
    if(dh > HORIZONTAL_TOLERANCE)
    {
        ROS_INFO("Moving base...");
        tf::Pose target_base_pose = robotPoseForCamPose(target_cam_pose);
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = m_world_frame_id;
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
            base_moved = true;
        } else {
            ROS_INFO("goal approach failed");
            m_move_base_client.cancelAllGoals();
            success = false;
        }
    }

    // lift torso if required
    bool torso_lifted = false;
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
            torso_lifted = true;
        } else {
            ROS_INFO("torso failed");
            m_lift_torso_client.cancelAllGoals();
            success = false;
        }
    }

    // Unlock head
    if(!base_moved && !torso_lifted) {
        ros::Duration(2.0).sleep(); // Give the head some time for movement
    }
    m_point_head_client.cancelGoal();

    return success;
}
