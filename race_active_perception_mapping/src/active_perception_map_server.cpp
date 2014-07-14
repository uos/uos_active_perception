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
#include <race_active_perception_mapping/PerceptionMap.h>

ActivePerceptionMapServer::ActivePerceptionMapServer() :
    m_node_handle("~"),
    m_node_handle_pub(),
    m_point_cloud_subscriber(m_node_handle_pub.subscribe(
                                 "cloud_in",
                                 1,
                                 &ActivePerceptionMapServer::pointCloudCb,
                                 this)),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/perception_map_marker", 10)),
    m_map_pub(m_node_handle_pub.advertise<race_active_perception_mapping::PerceptionMap>("perception_map", 1)),
    m_perception_map(0.01)
{
    m_node_handle.param("resolution"    , m_resolution    , 0.05);
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

    // Integrate point cloud
    m_perception_map.integratePointCloud(octomap_cloud, octomap::poseTfToOctomap(sensor_to_world_tf));

    // Publish map
    race_active_perception_mapping::PerceptionMap msg;
    msg.header.frame_id = m_world_frame_id;
    msg.header.stamp = cloud.header.stamp;
    octomap_msgs::binaryMapToMsgData(m_perception_map.getOccupancyMap(), msg.occupancy_data);
    octomap_msgs::binaryMapToMsgData(m_perception_map.getFringeMap(), msg.fringe_data);
    m_map_pub.publish(msg);

    // Publish rviz map visualization
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = m_resolution;
    marker.scale.y = m_resolution;
    marker.scale.z = m_resolution;
    marker.header.frame_id = m_world_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    // occupancy markers
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.ns = "occupancy";
    for(octomap::OcTree::leaf_iterator it = m_perception_map.getOccupancyMap().begin_leafs();
        it != m_perception_map.getOccupancyMap().end_leafs();
        it++)
    {
        if(m_perception_map.getOccupancyMap().isNodeOccupied(*it))
        {
            marker.points.push_back(octomap::pointOctomapToMsg(it.getCoordinate()));
        }
    }
    m_marker_pub.publish(marker);
    // fringe markers
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.2;
    marker.ns = "fringe";
    marker.points.clear();
    for(octomap::OcTree::leaf_iterator it = m_perception_map.getFringeMap().begin_leafs();
        it != m_perception_map.getFringeMap().end_leafs();
        it++)
    {
        marker.points.push_back(octomap::pointOctomapToMsg(it.getCoordinate()));
    }
    m_marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "active_perception_map_server");
    ActivePerceptionMapServer node;
    ROS_INFO("active_perception_map_server: Initialized!");
    ros::spin();
}
