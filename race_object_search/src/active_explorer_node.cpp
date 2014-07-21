#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <race_next_best_view/GetObservationCameraPoses.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "active_explorer");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    ros::Publisher marker_pub(nh.advertise<visualization_msgs::Marker>("/exploration_marker", 10));
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("move_base", true);
    // wait for the action server to come up
    while(!moveBaseClient.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    };

    while(ros::ok()) {
        ROS_INFO("retrieving pose candidates");
        race_next_best_view::GetObservationCameraPoses pose_candidates;
        pose_candidates.request.sample_size = 200;
        pose_candidates.request.roi.dimensions.x = 5;
        pose_candidates.request.roi.dimensions.y = 5;
        pose_candidates.request.roi.dimensions.z = 2;
        tf::quaternionTFToMsg(tf::createIdentityQuaternion(), pose_candidates.request.roi.pose_stamped.pose.orientation);
        pose_candidates.request.roi.pose_stamped.header.frame_id = "/base_footprint";
        pose_candidates.request.roi.pose_stamped.header.stamp = ros::Time::now();
        if(!ros::service::call("/get_observation_camera_poses", pose_candidates)) {
            ROS_ERROR("service call failed");
            ros::Duration(5).sleep();
            continue;
        }
        geometry_msgs::PoseStamped best_pose;
        double best_gain = -1.0;
        for(unsigned int i = 0; i < pose_candidates.response.camera_poses.size(); i++) {
            if(pose_candidates.response.information_gain[i] > best_gain) {
                best_gain = pose_candidates.response.information_gain[i];
                best_pose = pose_candidates.response.camera_poses[i];
            }
        }
        if(best_gain < 0) {
            ROS_WARN("no good next best views found");
            ros::Duration(5).sleep();
            continue;
        }

        tf::Pose best_pose_tf;
        tf::poseMsgToTF(best_pose.pose, best_pose_tf);

        // base_to_cam_tf: the base_footprint's pose as seen from the kinect
        tf::StampedTransform base_to_cam_tf;
        tf_listener.lookupTransform("/head_mount_kinect_ir_link", "/base_footprint", ros::Time(0), base_to_cam_tf);

        geometry_msgs::Pose best_pose_base;
        tf::poseTFToMsg(best_pose_tf * base_to_cam_tf, best_pose_base);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header = best_pose.header;
        goal.target_pose.pose = best_pose_base;

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

        ROS_INFO("Sending goal");
        moveBaseClient.sendGoal(goal);
        moveBaseClient.waitForResult();
        if(moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("goal reached with success");
            ros::Duration(6.0).sleep();
        } else {
            ROS_INFO("goal approach failed");
        }
    }
}
