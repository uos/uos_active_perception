#ifndef POINT_CLOUD_2_MOTION_FILTER_H
#define POINT_CLOUD_2_MOTION_FILTER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

class PointCloud2MotionFilter
{
public:
    PointCloud2MotionFilter();

private:
    ros::NodeHandle m_node_handle,
                    m_node_handle_pub;
    ros::Subscriber m_point_cloud_subscriber,
                    m_tf_subscriber;
    tf::TransformListener m_tf_listener;
    tf::StampedTransform m_last_tf;
    ros::Publisher m_point_cloud_pub;
    double m_block_time,
           m_max_distance,
           m_max_rotation;
    ros::Time m_last_movement;
    std::string m_static_frame;

    void pointCloudCb(sensor_msgs::PointCloud2 const & cloud);
};

#endif // POINT_CLOUD_2_MOTION_FILTER_H
