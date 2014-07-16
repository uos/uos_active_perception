#include "point_cloud_2_motion_filter.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

PointCloud2MotionFilter::PointCloud2MotionFilter() :
    m_node_handle("~"),
    m_node_handle_pub(),
    m_point_cloud_subscriber(m_node_handle_pub.subscribe(
                                 "cloud_in",
                                 1,
                                 &PointCloud2MotionFilter::pointCloudCb,
                                 this)),
    m_tf_listener(),
    m_point_cloud_pub(m_node_handle_pub.advertise<sensor_msgs::PointCloud2>("cloud_out", 1))
{
    m_node_handle.param("static_frame", m_static_frame, std::string("/map"));
    m_node_handle.param("block_time",   m_block_time,   3.0);
    m_node_handle.param("max_distance", m_max_distance, 0.01);
    m_node_handle.param("max_rotation", m_max_rotation, 0.002);

    m_last_movement = ros::Time::now();
}


void PointCloud2MotionFilter::pointCloudCb(sensor_msgs::PointCloud2 const & cloud)
{
    try
    {
        tf::StampedTransform current_tf;
        m_tf_listener.lookupTransform(m_static_frame, cloud.header.frame_id, ros::Time(), current_tf);
        // check distance and rotation
        if(current_tf.getOrigin().distance(m_last_tf.getOrigin()) > m_max_distance ||
           current_tf.getRotation().angleShortestPath(m_last_tf.getRotation()) > m_max_rotation)
        {
            ROS_INFO("point_cloud_2_motion_filter: blocked (motion)");
            m_last_movement = current_tf.stamp_;
            m_last_tf = current_tf;
            return;
        }
        // the last movement must be far enough in the past while the last data must be recent enough
        if((ros::Time::now() - m_last_movement) < ros::Duration(m_block_time) ||
           (ros::Time::now() - m_last_tf.stamp_) > ros::Duration(m_block_time))
        {
            ROS_INFO("point_cloud_2_motion_filter: blocked (recent movement or no recent data)");
            m_last_tf = current_tf;
            return;
        }
        // now we know that it is okay to open the filter
        m_last_tf = current_tf;
        m_point_cloud_pub.publish(cloud);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("point_cloud_2_motion_filter: Transform error of sensor data: "
                         << ex.what());
        return;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_2_motion_filter");
    PointCloud2MotionFilter node;
    ROS_INFO("point_cloud_2_motion_filter: Initialized!");
    ros::spin();
}
