#ifndef ACTIVE_PERCEPTION_MAP_SERVER_H
#define ACTIVE_PERCEPTION_MAP_SERVER_H

#include "active_perception_map.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <octomap_msgs/OctomapBinary.h>

#include <string>

class ActivePerceptionMapServer
{
public:
    ActivePerceptionMapServer();

private:
    ros::NodeHandle m_node_handle, m_node_handle_pub;
    ros::Subscriber m_point_cloud_subscriber;
    tf::TransformListener m_tf_listener;
    ros::Publisher m_occupancy_octree_pub,
                   m_fringe_octree_pub;
    ActivePerceptionMap m_perception_map;
    double m_resolution;
    std::string m_world_frame_id;

    octomap_msgs::OctomapBinary composeMsg(octomap::OcTree const & octree) const;

    // Callbacks
    void pointCloudCb(sensor_msgs::PointCloud2 const & cloud);
};

#endif // ACTIVE_PERCEPTION_MAP_SERVER_H
