#ifndef ASP_SPATIAL_REASONER_H
#define ASP_SPATIAL_REASONER_H

#include "ros/ros.h"
#include "asp_spatial_reasoning/GetBboxPercentUnseen.h"
#include "asp_msgs/BoundingBox.h"
#include "octomap_msgs/Octomap.h"
#include "tf/transform_listener.h"
#include "octomap/math/Vector3.h"

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

    octomap_msgs::Octomap getCurrentScene();
    std::vector<tf::Vector3> bboxVertices(const asp_msgs::BoundingBox&);

    // Callbacks
    bool getBboxPercentUnseenCb(asp_spatial_reasoning::GetBboxPercentUnseen::Request&,
                                asp_spatial_reasoning::GetBboxPercentUnseen::Response&);
};

#endif // ASP_SPATIAL_REASONER_H
