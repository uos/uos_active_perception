#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <race_next_best_view/GetObservationCameraPoses.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <string>
#include <algorithm>
#include <limits>

double const DRIVE_SPEED = 0.25; // [m/s]
double const TURN_SPEED = 1.0; // [rad/s]
double const LIFT_SPEED = 0.02; // [m/s]
double const HEAD_SPEED = 3.14; // [rad/s]
double const ACQUISITION_TIME = 7.0;
double const TRANSLATIONAL_TOLERANCE = 0.1;

/**
  Figures out the required torso position for a given camera height using magic numbers.
  */
double torso_position_for_cam_height(double const & cam_height)
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

geometry_msgs::PoseStamped frame_id_to_pose(std::string const & frame_id)
{
    geometry_msgs::PoseStamped ps;
    tf::quaternionTFToMsg(tf::createIdentityQuaternion(), ps.pose.orientation);
    ps.header.frame_id = frame_id;
    ps.header.stamp = ros::Time::now();
    return ps;
}

tf::Pose base_pose_for_cam_pose(tf::Pose const & target_cam_pose)
{
    // Calculate the base pose for this camera pose.
    // For pr2 the xy offset between cam and base is assumed to be 0 although it actually depends on cam orientation.
    tf::Vector3 base_position = target_cam_pose.getOrigin();
    base_position.setZ(0);
    tf::Vector3 xr = tf::Transform(target_cam_pose.getRotation(), tf::Vector3(0,0,0))(tf::Vector3(1,0,0));
    tf::Quaternion base_rotation;
    base_rotation.setRPY(0, 0, std::atan2(xr.getY(), xr.getX()));

    return tf::Pose(base_rotation, base_position);
}

bool is_move_base_required(tf::Pose const & current_cam_pose, tf::Pose const & target_cam_pose)
{
    double distance = std::sqrt(std::pow(current_cam_pose.getOrigin().getX() - target_cam_pose.getOrigin().getX(), 2) +
                                std::pow(current_cam_pose.getOrigin().getY() - target_cam_pose.getOrigin().getY(), 2));
    return distance > TRANSLATIONAL_TOLERANCE;
}

double estimate_move_base_time(tf::Pose const & current_base_pose, tf::Pose const & target_base_pose)
{
    if(is_move_base_required(current_base_pose, target_base_pose))
    {
        // generate path planner request
        nav_msgs::GetPlan get_plan_call;
        tf::poseTFToMsg(current_base_pose, get_plan_call.request.start.pose);
        get_plan_call.request.start.header.frame_id = "/map";
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

double estimate_exec_time
(
        tf::Pose const & current_base_pose,
        tf::Pose const & current_cam_pose,
        tf::Pose const & target_cam_pose)
{
    // initialize exec_time with constant data acquisition time
    double exec_time = ACQUISITION_TIME;

    // calculate the time needed to move_base
    if(is_move_base_required(current_cam_pose, target_cam_pose))
    {
        exec_time += estimate_move_base_time(current_base_pose, base_pose_for_cam_pose(target_cam_pose));
    }

    // calculate time needed to turn the camera (disregards that yaw may be already provided by move_base)
    exec_time += current_cam_pose.getRotation().angleShortestPath(target_cam_pose.getRotation()) / HEAD_SPEED;

    // calculate time needed to lift the torso
    double dz = std::abs(current_cam_pose.getOrigin().getZ() - target_cam_pose.getOrigin().getZ());
    if(dz > TRANSLATIONAL_TOLERANCE)
    {
        exec_time += dz / LIFT_SPEED;
    }

    return exec_time;
}

bool achieve_cam_pose
(
        tf::Pose const & current_base_pose,
        tf::Pose const & current_cam_pose,
        tf::Pose const & target_cam_pose)
{
    static ros::NodeHandle nh;
    static ros::Publisher marker_pub(nh.advertise<visualization_msgs::Marker>("/exploration_marker", 10));
    static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("move_base", true);
    static actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>
            pointHeadClient("/head_traj_controller/point_head_action", true);
    static actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction>
            liftTorsoClient("/torso_controller/position_joint_action", true);

    /*
    while(!moveBaseClient.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    };
    while(!pointHeadClient.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the point_head action server to come up");
    };
    */

    // move base if required
    if(is_move_base_required(current_cam_pose, target_cam_pose))
    {
        tf::Pose target_base_pose = base_pose_for_cam_pose(target_cam_pose);
        ROS_INFO("Moving base...");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        tf::poseTFToMsg(target_base_pose, goal.target_pose.pose);

        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.lifetime = ros::Duration();
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
        marker.pose = goal.target_pose.pose;
        marker.header = goal.target_pose.header;
        marker.id = 0;
        marker.ns = "active_explorer_debug";
        marker_pub.publish(marker);

        double timeout = estimate_move_base_time(current_base_pose, target_base_pose);
        if(timeout > 9000)
        {
            return false;
        }
        moveBaseClient.sendGoal(goal);
        moveBaseClient.waitForResult(ros::Duration(timeout + 5.0));
        if(moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("goal reached with success");
        } else {
            ROS_INFO("goal approach failed");
            moveBaseClient.cancelAllGoals();
            return false;
        }
    }

    if(std::abs(current_cam_pose.getOrigin().getZ() - target_cam_pose.getOrigin().getZ()) > TRANSLATIONAL_TOLERANCE)
    {
        ROS_INFO("Lifting torso...");
        pr2_controllers_msgs::SingleJointPositionGoal torso_goal;
        torso_goal.position = torso_position_for_cam_height(target_cam_pose.getOrigin().getZ());
        liftTorsoClient.sendGoal(torso_goal);
        liftTorsoClient.waitForResult(ros::Duration(7.0));
        if(liftTorsoClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("torso lifted with success");
        } else {
            ROS_INFO("torso failed");
            liftTorsoClient.cancelAllGoals();
            return false;
        }
    }

    ROS_INFO("Pointing head...");
    pr2_controllers_msgs::PointHeadGoal head_goal;
    head_goal.target.header.frame_id = "/map";
    head_goal.target.header.stamp = ros::Time::now();
    tf::pointTFToMsg(target_cam_pose * tf::Point(1e10,0,0), head_goal.target.point);

    pointHeadClient.sendGoal(head_goal);
    pointHeadClient.waitForResult(ros::Duration(3.0));
    if(pointHeadClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("head moved with success");
    } else {
        ROS_INFO("head movement failed");
        pointHeadClient.cancelAllGoals();
        return false;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "active_explorer");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    ros::Publisher marker_pub(nh.advertise<visualization_msgs::Marker>("/exploration_marker", 10));

    while(ros::ok())
    {
        std::vector<geometry_msgs::Pose> pose_candidates;
        std::vector<double> candidate_information_gain;

        ROS_INFO("retrieving pose candidates");
        {
            race_next_best_view::GetObservationCameraPoses pose_candidates_call;
            pose_candidates_call.request.sample_size = 200;
            pose_candidates_call.request.ray_skip = 0.98;
            //pose_candidates_call.request.roi.dimensions.x = 5;
            //pose_candidates_call.request.roi.dimensions.y = 5;
            //pose_candidates_call.request.roi.dimensions.z = 2;
            //pose_candidates_call.request.roi.pose_stamped = frame_id_to_pose("/base_footprint");
            if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call)) {
                ROS_ERROR("service call failed");
                ros::Duration(5).sleep();
                continue;
            }
            pose_candidates.insert(pose_candidates.end(),
                                   pose_candidates_call.response.camera_poses.begin(),
                                   pose_candidates_call.response.camera_poses.end());
            candidate_information_gain.insert(candidate_information_gain.end(),
                                              pose_candidates_call.response.information_gains.begin(),
                                              pose_candidates_call.response.information_gains.end());
        }

        ROS_INFO("retrieving pose candidates for fixed position");
        {
            geometry_msgs::PoseStamped current_head_pose = frame_id_to_pose("/base_footprint");
            current_head_pose.pose.position.z = 1.5;
            race_next_best_view::GetObservationCameraPoses pose_candidates_call;
            pose_candidates_call.request.sample_size = 100;
            pose_candidates_call.request.ray_skip = 0.98;
            pose_candidates_call.request.observation_position.header = current_head_pose.header;
            pose_candidates_call.request.observation_position.point = current_head_pose.pose.position;
            pose_candidates_call.request.lock_height = false;
            if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call)) {
                ROS_ERROR("service call failed");
                ros::Duration(5).sleep();
                continue;
            }
            pose_candidates.insert(pose_candidates.end(),
                                   pose_candidates_call.response.camera_poses.begin(),
                                   pose_candidates_call.response.camera_poses.end());
            candidate_information_gain.insert(candidate_information_gain.end(),
                                              pose_candidates_call.response.information_gains.begin(),
                                              pose_candidates_call.response.information_gains.end());
        }

        ROS_INFO("evaluating pose candidate utility values");

        // get the base_footprint's current pose in kinect frame and the world frame
        tf::StampedTransform cam_to_world_tf, base_to_world_tf;
        tf_listener.lookupTransform("/map", "/head_mount_kinect_ir_link", ros::Time(0), cam_to_world_tf);
        tf_listener.lookupTransform("/map", "/base_footprint", ros::Time(0), base_to_world_tf);
        // HACK for pr2 head cam
        cam_to_world_tf.setOrigin(tf::Vector3(base_to_world_tf.getOrigin().getX(),
                                              base_to_world_tf.getOrigin().getY(),
                                              1.5));

        std::vector<double> candidate_utilities(pose_candidates.size());
        tf::Pose best_cam_pose; best_cam_pose.setIdentity();
        double best_utility = 0;
        for(unsigned int i = 0; i < candidate_utilities.size(); i++)
        {
            tf::Pose pose_candidate_cam_tf;
            tf::poseMsgToTF(pose_candidates[i], pose_candidate_cam_tf);

            candidate_utilities[i] = candidate_information_gain[i] /
                                     estimate_exec_time(base_to_world_tf, cam_to_world_tf, pose_candidate_cam_tf);

            if(candidate_utilities[i] > best_utility)
            {
                best_cam_pose = pose_candidate_cam_tf;
                best_utility = candidate_utilities[i];
            }
        }

        if(best_utility == 0)
        {
            ROS_WARN("no good next best views found");
            ros::Duration(5).sleep();
            continue;
        }

        // do it
        achieve_cam_pose(base_to_world_tf, cam_to_world_tf, best_cam_pose);

        // wait for acquisition
        ros::Duration(7.0).sleep();
    }
}
