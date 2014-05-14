#ifndef ASP_SPATIAL_REASONER_H
#define ASP_SPATIAL_REASONER_H

#include "ros/ros.h"
#include "asp_spatial_reasoning/GetBboxOccupancy.h"
#include "asp_spatial_reasoning/GetObservationCameraPoses.h"
#include "asp_spatial_reasoning/GetObjectsToRemove.h"
#include "asp_msgs/BoundingBox.h"
#include "asp_msgs/CameraConstraints.h"
#include "octomap_msgs/Octomap.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "octomap/math/Vector3.h"
#include "octomap/octomap.h"
#include "perception_mapping.h"

#include <vector>
#include <string>

class AspSpatialReasoner
{
public:
    AspSpatialReasoner();

private:
    static const double PI = 3.14159265359;

    ros::NodeHandle m_node_handle, m_node_handle_pub;
    ros::Subscriber m_point_cloud_subscriber;
    ros::ServiceServer m_get_bbox_percent_unseen_server;
    ros::ServiceServer m_get_observation_camera_poses_server;
    ros::ServiceServer m_get_objects_to_remove_server;
    tf::TransformListener m_tf_listener;
    ros::Publisher m_marker_pub,
                   m_occupancy_octree_pub;
    asp_msgs::CameraConstraints m_camera_constraints;
    PerceptionMapping m_perception_mapping;
    double m_ray_casting_point_distance,
           m_resolution;
    int m_sample_size;
    std::string m_world_frame_id;

    /**
      Polls the current octomap from the octomap_server.
      */
    octomap_msgs::Octomap getCurrentScene() const;

    /**
      Returns a list of all bbox vertices in the same frame as the bbox pose.
      */
    std::vector<tf::Vector3> bboxVertices(asp_msgs::BoundingBox const &) const;

    /**
      Find the volume of the intersection of two axis-aligned bounding boxes.
      */
    double getIntersectionVolume(octomath::Vector3 const & box1min, octomath::Vector3 const & box1max,
                                 octomath::Vector3 const & box2min, octomath::Vector3 const & box2max) const;

    /**
      Find the axis-aligned bounds of any bounding box in a given frame.
      Inputs: frame, bbox.
      Outputs: min, max.
      */
    bool getAxisAlignedBounds(std::string const & frame, asp_msgs::BoundingBox const & bbox,
                              octomath::Vector3& min, octomath::Vector3& max) const;

    /**
      Calculate bbox occupancy in a given octomap.
      */
    asp_spatial_reasoning::GetBboxOccupancy::Response getBboxOccupancyInScene(octomap::OcTree const & octree,
                                                                              octomath::Vector3 const & min,
                                                                              octomath::Vector3 const & max) const;

    double getInformationGainForRegionRemoval(octomap::OcTree const & octree,
                                              std::list<octomath::Vector3> const & unknown_voxels,
                                              octomath::Vector3 const & remove_min,
                                              octomath::Vector3 const & remove_max,
                                              octomath::Vector3 const & cam_position) const;

    // Callbacks
    bool getBboxOccupancyCb(asp_spatial_reasoning::GetBboxOccupancy::Request&,
                            asp_spatial_reasoning::GetBboxOccupancy::Response&);

    bool getObservationCameraPosesCb(asp_spatial_reasoning::GetObservationCameraPoses::Request&,
                                     asp_spatial_reasoning::GetObservationCameraPoses::Response&);

    bool getObjectsToRemoveCb(asp_spatial_reasoning::GetObjectsToRemove::Request&,
                              asp_spatial_reasoning::GetObjectsToRemove::Response&);

    void pointCloudCb(sensor_msgs::PointCloud2 const & cloud);
};

#endif // ASP_SPATIAL_REASONER_H
