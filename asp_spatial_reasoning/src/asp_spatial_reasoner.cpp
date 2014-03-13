#include "asp_spatial_reasoner.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asp_spatial_reasoning/GetBboxOccupancy.h"
#include "octomap_msgs/GetOctomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "octomap/math/Vector3.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

#include <sstream>
#include <vector>
#include <limits>
#include <algorithm>

AspSpatialReasoner::AspSpatialReasoner() :
    m_node_handle("~"),
    m_get_octomap_client(m_node_handle.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary")),
    m_get_bbox_percent_unseen_server(m_node_handle.advertiseService("/get_bbox_occupancy",
                                                                    &AspSpatialReasoner::getBboxOccupancyCb,
                                                                    this)),
    m_tf_listener(),
    m_marker_pub(m_node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10000))
{}

octomap_msgs::Octomap AspSpatialReasoner::getCurrentScene()
{
    octomap_msgs::GetOctomap getOctomap;
    m_get_octomap_client.call(getOctomap);
    return getOctomap.response.map;
}

std::vector<tf::Vector3> AspSpatialReasoner::bboxVertices(const asp_msgs::BoundingBox& bbox)
{
    // Construct a vector of bounding box vertices relative to the bounding box pose
    std::vector<tf::Vector3> bbox_points(8);
    bbox_points[0] = tf::Vector3(-bbox.dimensions.x/2, -bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[1] = tf::Vector3(-bbox.dimensions.x/2, -bbox.dimensions.y/2, -bbox.dimensions.z/2);
    bbox_points[2] = tf::Vector3(-bbox.dimensions.x/2, +bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[3] = tf::Vector3(-bbox.dimensions.x/2, +bbox.dimensions.y/2, -bbox.dimensions.z/2);
    bbox_points[4] = tf::Vector3(+bbox.dimensions.x/2, -bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[5] = tf::Vector3(+bbox.dimensions.x/2, -bbox.dimensions.y/2, -bbox.dimensions.z/2);
    bbox_points[6] = tf::Vector3(+bbox.dimensions.x/2, +bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[7] = tf::Vector3(+bbox.dimensions.x/2, +bbox.dimensions.y/2, -bbox.dimensions.z/2);
    // Transform points to bbox frame
    tf::Quaternion tf_orientation;
    tf::Point tf_position;
    tf::quaternionMsgToTF(bbox.pose_stamped.pose.orientation, tf_orientation);
    tf::pointMsgToTF(bbox.pose_stamped.pose.position, tf_position);
    tf::Transform transform(tf_orientation, tf_position);
    for(std::vector<tf::Vector3>::iterator it = bbox_points.begin(); it != bbox_points.end(); ++it)
    {
        *it = transform(*it);
    }
    return bbox_points;
}

double AspSpatialReasoner::getIntersectionVolume(octomath::Vector3& box1min, octomath::Vector3& box1max,
                                                 octomath::Vector3& box2min, octomath::Vector3& box2max)
{
//    static int i = 0;
    octomath::Vector3 max, min;
    // intersection min is the maximum of the two boxes' mins
    // intersection max is the minimum of the two boxes' maxes
    max.x() = std::min(box1max.x(), box2max.x());
    max.y() = std::min(box1max.y(), box2max.y());
    max.z() = std::min(box1max.z(), box2max.z());
    min.x() = std::max(box1min.x(), box2min.x());
    min.y() = std::max(box1min.y(), box2min.y());
    min.z() = std::max(box1min.z(), box2min.z());
    // make sure that max is actually larger in each dimension
    if(max.x() < min.x()) return 0.0;
    if(max.y() < min.y()) return 0.0;
    if(max.z() < min.z()) return 0.0;
//    // Send a marker
//    visualization_msgs::Marker marker;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.type = visualization_msgs::Marker::CUBE;
//    marker.lifetime = ros::Duration();
//    marker.scale.x = max.x() - min.x();
//    marker.scale.y = max.y() - min.y();
//    marker.scale.z = max.z() - min.z();
//    marker.color.r = 1.0;
//    marker.color.g = 1.0;
//    marker.color.b = 1.0;
//    marker.color.a = 0.5;
//    marker.pose.position.x = (min.x() + max.x()) / 2;
//    marker.pose.position.y = (min.y() + max.y()) / 2;
//    marker.pose.position.z = (min.z() + max.z()) / 2;
//    marker.header.frame_id = "/map";
//    marker.header.stamp = ros::Time::now();
//    marker.id = i++;
//    marker.ns = "asp_spatial_reasoner_debug";
//    m_marker_pub.publish(marker);
    // return the volume
    return (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
}

bool AspSpatialReasoner::getBboxOccupancyCb(asp_spatial_reasoning::GetBboxOccupancy::Request &req,
                                            asp_spatial_reasoning::GetBboxOccupancy::Response &res)
{
    octomap_msgs::Octomap map_msg = getCurrentScene();
    if(map_msg.data.size() == 0)
    {
        ROS_ERROR("asp_spatial_reasoner: Could not retrieve current octomap");
        return false;
    }
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map_msg));
    // Find the axis aligned bounding box in the octomap
    float inf = std::numeric_limits<float>::infinity();
    octomath::Vector3 max(-inf, -inf, -inf), min(inf, inf, inf);
    std::vector<tf::Vector3> bbox_vertices = bboxVertices(req.bbox);
    if(!m_tf_listener.waitForTransform(map_msg.header.frame_id,
                                       req.bbox.pose_stamped.header.frame_id,
                                       req.bbox.pose_stamped.header.stamp,
                                       ros::Duration(1)))
    {
        ROS_ERROR_STREAM("asp_spatial_reasoner: Timed out while waiting for transform from " <<
                          req.bbox.pose_stamped.header.frame_id << " to " <<
                          map_msg.header.frame_id);
    }
    for(std::vector<tf::Vector3>::iterator it = bbox_vertices.begin(); it != bbox_vertices.end(); ++it)
    {
        geometry_msgs::PointStamped pin, pout;
        pin.header = req.bbox.pose_stamped.header;
        pin.point.x = it->x();
        pin.point.y = it->y();
        pin.point.z = it->z();
        m_tf_listener.transformPoint(map_msg.header.frame_id, pin, pout);
        if(pout.point.x < min.x()) min.x() = pout.point.x;
        if(pout.point.y < min.y()) min.y() = pout.point.y;
        if(pout.point.z < min.z()) min.z() = pout.point.z;
        if(pout.point.x > max.x()) max.x() = pout.point.x;
        if(pout.point.y > max.y()) max.y() = pout.point.y;
        if(pout.point.z > max.z()) max.z() = pout.point.z;
    }
    double total_volume = (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
    double free_volume = 0.0, occupied_volume = 0.0;
    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max), end = octree->end_leafs_bbx();
        it != end; ++it)
    {
        double side_len_half = it.getSize() / 2.0;
        octomath::Vector3 vox_min = it.getCoordinate(), vox_max = it.getCoordinate();
        vox_min.x() -= side_len_half;
        vox_min.y() -= side_len_half;
        vox_min.z() -= side_len_half;
        vox_max.x() += side_len_half;
        vox_max.y() += side_len_half;
        vox_max.z() += side_len_half;
        double v = getIntersectionVolume(min, max, vox_min, vox_max);
        if(it->getOccupancy() > 0.5) // TODO: Hack!
        {
            occupied_volume += v;
        }
        else
        {
            free_volume += v;
        }
    }
    res.free = free_volume / total_volume;
    res.occupied = occupied_volume / total_volume;
    res.unknown = 1.0 - res.free - res.occupied;
    delete octree;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "asp_spatial_reasoner");
    AspSpatialReasoner node;
    ROS_INFO("asp_spatial_reasoner: Initialized!");
    ros::spin();
}
