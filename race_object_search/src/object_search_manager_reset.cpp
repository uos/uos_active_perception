#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "actionlib/client/simple_action_client.h"
#include "uos_active_perception_msgs/ResetVolumes.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_object_search_manager_reset");
    ros::NodeHandle n("~");

    // Wait for subscribers to connect
    ros::Duration(1).sleep();

    // Create the request
    uos_active_perception_msgs::BoundingBox box;
    box.pose_stamped.header.frame_id = "/map";
    box.pose_stamped.header.stamp = ros::Time::now();
    box.pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    n.param("x", box.pose_stamped.pose.position.x, 5.5);
    n.param("y", box.pose_stamped.pose.position.y, 10.0);
    n.param("z", box.pose_stamped.pose.position.z, 1.0);
    n.param("xdim", box.dimensions.x, 1.0);
    n.param("ydim", box.dimensions.y, 1.0);
    n.param("zdim", box.dimensions.z, 1.0);

    ros::ServiceClient c = n.serviceClient<uos_active_perception_msgs::ResetVolumes>("/reset_volumes");
    uos_active_perception_msgs::ResetVolumes srv;
    srv.request.volumes.push_back(box);
    c.waitForExistence();
    ROS_INFO("Sending goal...");
    c.call(srv);
    ROS_INFO("completed!");


    ros::spinOnce();
}

