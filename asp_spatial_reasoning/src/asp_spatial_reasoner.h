#ifndef ASP_SPATIAL_REASONER_H
#define ASP_SPATIAL_REASONER_H

#include "ros/ros.h"
#include "asp_spatial_reasoning/GetBboxOccupancy.h"
#include "asp_msgs/BoundingBox.h"
#include "octomap_msgs/Octomap.h"
#include "tf/transform_listener.h"
#include "octomap/math/Vector3.h"
#include "octomap/octomap.h"

#include <vector>

class AspSpatialReasoner
{
public:
    AspSpatialReasoner();

private:
    ros::NodeHandle m_node_handle;
    ros::ServiceClient m_get_octomap_client;
    ros::ServiceServer m_get_bbox_percent_unseen_server;
    tf::TransformListener m_tf_listener;
    ros::Publisher m_marker_pub;

    /**
      Polls the current octomap from the octomap_server.
      */
    octomap_msgs::Octomap getCurrentScene();

    /**
      Returns a list of all bbox vertices in the same frame as the bbox pose.
      */
    std::vector<tf::Vector3> bboxVertices(const asp_msgs::BoundingBox&);

    /**
      Find the volume of the intersection of two axis-aligned bounding boxes.
      */
    double getIntersectionVolume(octomath::Vector3& box1min, octomath::Vector3& box1max,
                                 octomath::Vector3& box2min, octomath::Vector3& box2max);

    /**
      Calculate bbox occupancy in a given octomap.
      */
    asp_spatial_reasoning::GetBboxOccupancy::Response getBboxOccupancyInScene(octomap::OcTree& octree,
                                                                              octomath::Vector3& min,
                                                                              octomath::Vector3& max);

    // Callbacks
    bool getBboxOccupancyCb(asp_spatial_reasoning::GetBboxOccupancy::Request&,
                            asp_spatial_reasoning::GetBboxOccupancy::Response&);
};

#endif // ASP_SPATIAL_REASONER_H
