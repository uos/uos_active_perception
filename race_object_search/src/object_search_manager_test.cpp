#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "actionlib/client/simple_action_client.h"
#include "race_object_search/ObserveVolumesAction.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_object_search_manager_test");
    ros::NodeHandle n("~");
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/race_object_search_manager_test", 10);

    // Wait for subscribers to connect
    ros::Duration(1).sleep();

    // Create the request
    race_msgs::BoundingBox box;
    box.pose_stamped.header.frame_id = "/map";
    box.pose_stamped.header.stamp = ros::Time::now();
    box.pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    std::vector<race_msgs::BoundingBox> boxes(3, box);

    // table1
    boxes[0].pose_stamped.pose.position.x = 7.7;
    boxes[0].pose_stamped.pose.position.y = 11.4;
    boxes[0].pose_stamped.pose.position.z =  1.0;
    boxes[0].dimensions.x = 1.0;
    boxes[0].dimensions.y = 1.0;
    boxes[0].dimensions.z = 1.0;

    // table2
    boxes[1].pose_stamped.pose.position.x = 10.4;
    boxes[1].pose_stamped.pose.position.y = 11.4;
    boxes[1].pose_stamped.pose.position.z =  1.0;
    boxes[1].dimensions.x = 1.0;
    boxes[1].dimensions.y = 1.0;
    boxes[1].dimensions.z = 1.0;

    // counter
    boxes[2].pose_stamped.pose.position.x = 5.2;
    boxes[2].pose_stamped.pose.position.y = 10.0;
    boxes[2].pose_stamped.pose.position.z =  1.0;
    boxes[2].dimensions.x = 1.0;
    boxes[2].dimensions.y = 1.6;
    boxes[2].dimensions.z = 1.0;

    // Send a marker
    for(size_t i = 0; i < boxes.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.lifetime = ros::Duration();
        marker.scale.x = boxes[i].dimensions.x;
        marker.scale.y = boxes[i].dimensions.y;
        marker.scale.z = boxes[i].dimensions.z;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.pose = boxes[i].pose_stamped.pose;
        marker.header = boxes[i].pose_stamped.header;
        marker.id = i;
        marker.ns = "test";
        marker_pub.publish(marker);
    }


    actionlib::SimpleActionClient<race_object_search::ObserveVolumesAction> ac("/observe_volumes", true);
    race_object_search::ObserveVolumesGoal goal;
    goal.roi.insert(goal.roi.end(), boxes.begin(), boxes.end());
    goal.p = std::vector<float>(goal.roi.size(), 1.0 / goal.roi.size());
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

