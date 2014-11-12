#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "actionlib/client/simple_action_client.h"
#include "race_object_search/ObserveVolumesAction.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_object_search_manager_test");
    ros::NodeHandle n("~");
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/race_object_search_manager_test", 1);

    // Wait for subscribers to connect
    ros::Duration(1).sleep();

    // Create the request
    race_msgs::BoundingBox box;
    box.pose_stamped.header.frame_id = "/map";
    box.pose_stamped.header.stamp = ros::Time::now();
    box.pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    n.param("x", box.pose_stamped.pose.position.x, 10.4);
    n.param("y", box.pose_stamped.pose.position.y, 11.4);
    n.param("z", box.pose_stamped.pose.position.z,  1.0);
    n.param("xdim", box.dimensions.x, 1.0);
    n.param("ydim", box.dimensions.y, 1.0);
    n.param("zdim", box.dimensions.z, 1.0);

    // Send a marker
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();
    marker.scale.x = box.dimensions.x;
    marker.scale.y = box.dimensions.y;
    marker.scale.z = box.dimensions.z;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.pose = box.pose_stamped.pose;
    marker.header = box.pose_stamped.header;
    marker.id = 7;
    marker.ns = "test";
    marker_pub.publish(marker);

    actionlib::SimpleActionClient<race_object_search::ObserveVolumesAction> ac("/observe_volumes", true);
    race_object_search::ObserveVolumesGoal goal;
    goal.roi.push_back(box);
    ac.waitForServer();
    ROS_INFO("Sending goal...");
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("completed!");
    }
    else
    {
        ROS_INFO("failed!");
    }

    ros::spinOnce();
}

