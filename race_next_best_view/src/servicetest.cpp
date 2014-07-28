#include "ros/ros.h"
#include "race_next_best_view/GetObservationCameraPoses.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "race_next_best_view/GetBboxOccupancy.h"
#include "race_next_best_view/ResetVolumes.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_object_search_servicetest");
    ros::NodeHandle n("~");
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/servicetest_marker", 1);

    // Wait for subscribers to connect
    ros::Duration(1).sleep();

    // Create the request
    race_msgs::BoundingBox box;
    box.pose_stamped.header.frame_id = "/map";
    box.pose_stamped.header.stamp = ros::Time::now();
    box.pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    n.param("x", box.pose_stamped.pose.position.x, 5.5);
    n.param("y", box.pose_stamped.pose.position.y, 10.0);
    n.param("z", box.pose_stamped.pose.position.z, 1.0);
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
    marker.ns = "servicetest";
    marker_pub.publish(marker);

    // Send request
    race_next_best_view::GetBboxOccupancy get_bbox;
    get_bbox.request.bbox = box;
    if(ros::service::call("/get_bbox_occupancy", get_bbox))
    {
        ROS_INFO_STREAM("service call get_bbox_occupancy successful: " << get_bbox.response.free << " free / " << get_bbox.response.occupied << " occupied / " << get_bbox.response.unknown << " unknown");
    }
    else
    {
        ROS_ERROR("service call failed!");
    }

    // Send request
    race_next_best_view::GetObservationCameraPoses get_ocp;
    get_ocp.request.roi.push_back(box);
    get_ocp.request.sample_size = 100;
    get_ocp.request.ray_skip = 0.8;
    if(ros::service::call("/get_observation_camera_poses", get_ocp))
    {
        ROS_INFO_STREAM("service call get_observation_camera_poses successful.");
    }
    else
    {
        ROS_ERROR("service call failed!");
    }
    
    // Send request
    race_next_best_view::ResetVolumes reset_volumes_call;
    reset_volumes_call.request.volumes.push_back(box);
    if(ros::service::call("/reset_volumes", reset_volumes_call))
    {
        ROS_INFO_STREAM("service call reset_volumes successful.");
    }
    else
    {
        ROS_ERROR("service call failed!");
    }

    ros::spinOnce();
}

