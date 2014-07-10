#include "active_perception_map_server.h"

#include "active_perception_map.h"

#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap_msgs/OctomapBinary.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/Marker.h>

ActivePerceptionMapServer::ActivePerceptionMapServer() :
    m_node_handle("~"),
    m_node_handle_pub(),
    m_point_cloud_subscriber(m_node_handle_pub.subscribe(
                                 "cloud_in",
                                 1,
                                 &ActivePerceptionMapServer::pointCloudCb,
                                 this)),
    m_tf_listener(),
    m_occupancy_octree_pub(m_node_handle_pub.advertise<octomap_msgs::OctomapBinary>("occupancy_octree", 1)),
    m_fringe_octree_pub(m_node_handle_pub.advertise<octomap_msgs::OctomapBinary>("fringe_octree", 1)),
    m_perception_map(0.01)
{
    m_node_handle.param("resolution"    , m_resolution    , 0.1);
    m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));

    m_perception_map.setResolution(m_resolution);
}

octomap_msgs::OctomapBinary ActivePerceptionMapServer::composeMsg(octomap::OcTree const & octree) const
{
    octomap_msgs::OctomapBinary msg;
    octomap_msgs::binaryMapToMsgData(octree, msg.data);
    msg.header.frame_id = m_world_frame_id;
    msg.header.stamp = ros::Time::now();
    return msg;
}

void ActivePerceptionMapServer::pointCloudCb(sensor_msgs::PointCloud2 const & cloud)
{
    // Find sensor origin
    tf::StampedTransform sensor_to_world_tf;
    try
    {
        m_tf_listener.waitForTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, ros::Duration(2.0));
        m_tf_listener.lookupTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, sensor_to_world_tf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("active_perception_map_server: Transform error of sensor data: "
                         << ex.what()
                         << ", cannot integrate data.");
        return;
    }

    // Ugly converter cascade to get octomap_cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    octomap::Pointcloud octomap_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    octomap::pointcloudPCLToOctomap(pcl_cloud, octomap_cloud);
    m_perception_map.integratePointCloud(octomap_cloud, octomap::poseTfToOctomap(sensor_to_world_tf));
    m_occupancy_octree_pub.publish(composeMsg(m_perception_map.getOccupancyMap()));
    m_fringe_octree_pub.publish(composeMsg(m_perception_map.getFringeMap()));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "active_perception_map_server");
    ActivePerceptionMapServer node;
    ROS_INFO("active_perception_map_server: Initialized!");
    ros::spin();
}
