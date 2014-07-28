#ifndef NEXT_BEST_VIEW_NODE_H
#define NEXT_BEST_VIEW_NODE_H

#include "active_perception_map.h"
#include "camera_constraints.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <race_next_best_view/GetBboxOccupancy.h>
#include <race_next_best_view/GetObjectsToRemove.h>
#include <race_next_best_view/GetObservationCameraPoses.h>
#include <race_next_best_view/ResetVolumes.h>

#include <string>
#include <vector>

class NextBestViewNode
{
public:
    NextBestViewNode();

private:
    static const double PI = 3.14159265359;

    ros::NodeHandle m_node_handle, m_node_handle_pub;
    ros::Subscriber m_point_cloud_subscriber;
    ros::ServiceServer m_get_bbox_percent_unseen_server;
    ros::ServiceServer m_get_observation_camera_poses_server;
    ros::ServiceServer m_get_objects_to_remove_server;
    ros::ServiceServer m_reset_volumes_server;
    tf::TransformListener m_tf_listener;
    ros::Publisher m_marker_pub;
    CameraConstraints m_camera_constraints;
    ActivePerceptionMap m_perception_map;
    double m_resolution, m_ray_skip;
    std::string m_world_frame_id;

    static double getIntersectionVolume(const octomath::Vector3 &box1min, const octomath::Vector3 &box1max,
                                        const octomath::Vector3 &box2min, const octomath::Vector3 &box2max);
    static std::vector<tf::Vector3> bboxVertices(race_msgs::BoundingBox const & bbox);

    bool getAxisAlignedBounds(race_msgs::BoundingBox const & bbox,
                              octomath::Vector3 & min,
                              octomath::Vector3 & max) const;

    std::vector<tf::Transform> sampleObservationSpace(std::vector<octomap::point3d> const & points_of_interest,
                                                      int sample_size) const;

    // Callbacks
    void pointCloudCb(sensor_msgs::PointCloud2 const & cloud);

    bool getBboxOccupancyCb(race_next_best_view::GetBboxOccupancy::Request&,
                            race_next_best_view::GetBboxOccupancy::Response&);

    bool getObservationCameraPosesCb(race_next_best_view::GetObservationCameraPoses::Request&,
                                     race_next_best_view::GetObservationCameraPoses::Response&);

    bool getObjectsToRemoveCb(race_next_best_view::GetObjectsToRemove::Request&,
                              race_next_best_view::GetObjectsToRemove::Response&);

    bool resetVolumesCb(race_next_best_view::ResetVolumes::Request&,
                        race_next_best_view::ResetVolumes::Response&);
};

#endif // NEXT_BEST_VIEW_NODE_H
