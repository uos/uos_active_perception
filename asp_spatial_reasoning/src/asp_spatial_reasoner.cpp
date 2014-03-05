#include "asp_spatial_reasoner.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asp_spatial_reasoning/GetBboxPercentUnseen.h"
#include "octomap_msgs/GetOctomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "octomap/math/Vector3.h"
#include "tf/transform_datatypes.h"

#include <sstream>
#include <vector>
#include <limits>

AspSpatialReasoner::AspSpatialReasoner() :
    m_node_handle("~"),
    m_get_octomap_client(m_node_handle.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary")),
    m_get_bbox_percent_unseen_server(m_node_handle.advertiseService("/get_bbox_percent_unseen",
                                                                    &AspSpatialReasoner::getBboxPercentUnseenCb,
                                                                    this)),
    m_tf_listener()
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

bool AspSpatialReasoner::getBboxPercentUnseenCb(asp_spatial_reasoning::GetBboxPercentUnseen::Request &req,
                                                asp_spatial_reasoning::GetBboxPercentUnseen::Response &res)
{
    octomap_msgs::Octomap map_msg = getCurrentScene();
    if(map_msg.data.size() == 0)
    {
        ROS_ERROR("asp_spatial_reasoner: Could not retrieve current octomap");
        return false;
    }
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map_msg));
    unsigned int depth = octree->getTreeDepth();
    ROS_INFO_STREAM("Tree depth is " << depth);
    // Find the axis aligned bounding box in the octomap
    unsigned short maxkeyval = std::numeric_limits<unsigned short>::max();
    octomap::OcTreeKey max(0, 0, 0), min(maxkeyval, maxkeyval, maxkeyval);
    std::vector<tf::Vector3> bbox_vertices = bboxVertices(req.bbox);
    for(std::vector<tf::Vector3>::iterator it = bbox_vertices.begin(); it != bbox_vertices.end(); ++it)
    {
        geometry_msgs::PointStamped pin, pout;
        pin.header = req.bbox.pose_stamped.header;
        pin.point.x = it->x();
        pin.point.y = it->y();
        pin.point.z = it->z();
        m_tf_listener.transformPoint(map_msg.header.frame_id, pin, pout);
        octomap::OcTreeKey key = octree->coordToKey(pout.point.x, pout.point.y, pout.point.z, depth);
        if(key[0] < min[0]) min[0] = key[0];
        if(key[1] < min[1]) min[1] = key[1];
        if(key[2] < min[2]) min[2] = key[2];
        if(key[0] > max[0]) max[0] = key[0];
        if(key[1] > max[1]) max[1] = key[1];
        if(key[2] > max[2]) max[2] = key[2];
    }
    double total_size_x = (max[0] - min[0]) * octree->getNodeSize(depth);
    double total_size_y = (max[1] - min[1]) * octree->getNodeSize(depth);
    double total_size_z = (max[2] - min[2]) * octree->getNodeSize(depth);
    double total_volume = total_size_x * total_size_y * total_size_z;
    double observed_volume = 0.0;
    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max), end = octree->end_leafs_bbx();
        it != end; ++it)
    {
        double side_len = it.getSize();
        observed_volume += side_len * side_len * side_len;
    }
    res.percentUnseen = (total_volume - observed_volume) / total_volume;
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
