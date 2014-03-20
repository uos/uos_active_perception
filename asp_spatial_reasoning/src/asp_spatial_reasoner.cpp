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
#include "random_numbers/random_numbers.h"

#include <sstream>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

AspSpatialReasoner::AspSpatialReasoner() :
    m_node_handle("~"),
    m_get_octomap_client(m_node_handle.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary")),
    m_get_bbox_percent_unseen_server(m_node_handle.advertiseService(
                                         "/get_bbox_occupancy",
                                         &AspSpatialReasoner::getBboxOccupancyCb,
                                         this)),
    m_get_observation_camera_poses_server(m_node_handle.advertiseService(
                                              "/get_observation_camera_poses",
                                              &AspSpatialReasoner::getObservationCameraPosesCb,
                                              this)),
    m_tf_listener(),
    m_marker_pub(m_node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10000)),
    m_camera_constraints()
{
    // Set camera constraints from parameters (Defaults: Kinect on [insert robot])
    m_node_handle.param("height_min", m_camera_constraints.height_min, 0.5);
    m_node_handle.param("height_max", m_camera_constraints.height_max, 1.5);
    m_node_handle.param("pitch_min" , m_camera_constraints.pitch_min , -0.174532925);
    m_node_handle.param("pitch_max" , m_camera_constraints.pitch_max , 0.174532925);
    m_node_handle.param("range_min" , m_camera_constraints.range_min , 0.4);
    m_node_handle.param("range_max" , m_camera_constraints.range_max , 5.0);
    m_node_handle.param("hfov"      , m_camera_constraints.hfov      , 1.01229097);
    m_node_handle.param("vfov"      , m_camera_constraints.vfov      , 0.767944871);

    m_node_handle.param("ray_casting_point_distance", m_ray_casting_point_distance, 0.1);

}

octomap_msgs::Octomap AspSpatialReasoner::getCurrentScene() const
{
    octomap_msgs::GetOctomap getOctomap;
    m_get_octomap_client.call(getOctomap);
    return getOctomap.response.map;
}

std::vector<tf::Vector3> AspSpatialReasoner::bboxVertices(const asp_msgs::BoundingBox& bbox) const
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

double AspSpatialReasoner::getIntersectionVolume(const octomath::Vector3 &box1min, const octomath::Vector3 &box1max,
                                                 const octomath::Vector3 &box2min, const octomath::Vector3 &box2max) const
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

bool AspSpatialReasoner::getAxisAlignedBounds(std::string const & frame_id, asp_msgs::BoundingBox const & bbox,
                                              octomath::Vector3& min, octomath::Vector3& max) const
{
    float inf = std::numeric_limits<float>::infinity();
    min.x() =  inf; min.y() =  inf; min.z() =  inf;
    max.x() = -inf; max.y() = -inf; max.z() = -inf;

    std::vector<tf::Vector3> bbox_vertices = bboxVertices(bbox);
    if(!m_tf_listener.waitForTransform(frame_id,
                                       bbox.pose_stamped.header.frame_id,
                                       bbox.pose_stamped.header.stamp,
                                       ros::Duration(1)))
    {
        ROS_ERROR_STREAM("asp_spatial_reasoner: Timed out while waiting for transform from " <<
                          bbox.pose_stamped.header.frame_id << " to " <<
                          frame_id);
        return false;
    }
    for(std::vector<tf::Vector3>::iterator it = bbox_vertices.begin(); it != bbox_vertices.end(); ++it)
    {
        geometry_msgs::PointStamped pin, pout;
        pin.header = bbox.pose_stamped.header;
        pin.point.x = it->x();
        pin.point.y = it->y();
        pin.point.z = it->z();
        m_tf_listener.transformPoint(frame_id, pin, pout);
        if(pout.point.x < min.x()) min.x() = pout.point.x;
        if(pout.point.y < min.y()) min.y() = pout.point.y;
        if(pout.point.z < min.z()) min.z() = pout.point.z;
        if(pout.point.x > max.x()) max.x() = pout.point.x;
        if(pout.point.y > max.y()) max.y() = pout.point.y;
        if(pout.point.z > max.z()) max.z() = pout.point.z;
    }
    return true;
}

asp_spatial_reasoning::GetBboxOccupancy::Response AspSpatialReasoner::getBboxOccupancyInScene(
        const octomap::OcTree &octree, const octomath::Vector3 &min, const octomath::Vector3 &max) const
{
    double total_volume = (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
    double free_volume = 0.0, occupied_volume = 0.0;
    for(octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(min,max), end = octree.end_leafs_bbx();
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
    asp_spatial_reasoning::GetBboxOccupancy::Response res;
    res.free = free_volume / total_volume;
    res.occupied = occupied_volume / total_volume;
    res.unknown = 1.0 - res.free - res.occupied;
    return res;
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
    octomath::Vector3 min, max;
    getAxisAlignedBounds(map_msg.header.frame_id, req.bbox, min, max);
    res = getBboxOccupancyInScene(*octree, min, max);
    delete octree;
    return true;
}

bool AspSpatialReasoner::getObservationCameraPosesCb(asp_spatial_reasoning::GetObservationCameraPoses::Request& req,
                                                     asp_spatial_reasoning::GetObservationCameraPoses::Response& res)
{
    // Retrieve octomap of current scene
    octomap_msgs::Octomap map_msg = getCurrentScene();
    if(map_msg.data.size() == 0)
    {
        ROS_ERROR("asp_spatial_reasoner: Could not retrieve current octomap");
        return false;
    }
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map_msg));

    // Get axis-aligned bounds
    octomath::Vector3 roi_min, roi_max;
    getAxisAlignedBounds(map_msg.header.frame_id, req.roi, roi_min, roi_max);

    // Remember current state of ROI occlusion
    double old_occlusion = getBboxOccupancyInScene(*octree, roi_min, roi_max).unknown;

    // Gather position samples in space around ROI center
    int n_samples; m_node_handle.param("n_samples", n_samples, 500);
    std::vector<geometry_msgs::Pose> samples();
    octomath::Vector3 roi_center((roi_min.x() + roi_max.x()) / 2.0,
                                 (roi_min.y() + roi_max.y()) / 2.0,
                                 (roi_min.z() + roi_max.z()) / 2.0);
    random_numbers::RandomNumberGenerator rng;
    double max_range_increment = m_camera_constraints.range_max - m_camera_constraints.range_min;
    double max_height_increment = m_camera_constraints.height_max - m_camera_constraints.height_min;
    for(int i = 0; i < n_samples; ++i)
    {
        // Create random position within feasible range and height
        double direction = rng.uniform01() * 2.0 * PI;
        double distance = m_camera_constraints.range_min + rng.uniform01() * max_range_increment;
        // TODO: Add more distance to account for the ROI volume itself
        tf::Vector3 position(roi_center.x() + distance * std::cos(direction),
                             roi_center.y() + distance * std::sin(direction),
                             m_camera_constraints.height_min + rng.uniform01() * max_height_increment);
                             // TODO: Above line will go wrong if height constraints are not in octomap frame

        // Point the created poses towards the ROI center
        tf::Vector3 forward_axis(1, 0, 0);
        tf::Vector3 roi_direction(roi_center.x() - position.getX(),
                                  roi_center.y() - position.getY(),
                                  roi_center.z() - position.getZ());

        tf::Quaternion orientation(tf::tfCross(forward_axis, roi_direction),
                                   tf::tfAngle(forward_axis, roi_direction));
        tf::Transform transform(orientation, position);

        // TODO: Filter poses that are occupied or have infeasible pitch.
        //       Attempt to correct infeasible pitch while keeping ROI in vfov.

        // Project a hallucinated wall into the scene
        octomap::OcTree hallucination(*octree);
        octomath::Vector3 cam_pos(position.getX(), position.getY(), position.getZ());
        double azimuth_start = -m_camera_constraints.hfov / 2.0;
        double azimuth_end   =  m_camera_constraints.hfov / 2.0;
        double inclination_start = -m_camera_constraints.vfov / 2.0;
        double inclination_end   =  m_camera_constraints.vfov / 2.0;
        double r = m_camera_constraints.range_max;
        // TODO: Constrain the ray casting region to the ROI
        double angular_increment = std::acos(1.0 - (std::pow(m_ray_casting_point_distance, 2) /
                                                   (2.0 * r*r)));
        for(double azimuth = azimuth_start; azimuth < azimuth_end; azimuth += angular_increment)
        {
            for(double inclination = inclination_start; inclination < inclination_end; inclination += angular_increment)
            {
                // Construct target point in virtual camera frame
                float y = r * std::sin(azimuth);
                float z = r * std::sin(inclination);
                float x = std::sqrt(r*r - y*y - z*z);
                // Transform to scene frame
                tf::Vector3 tfTarget = transform(tf::Vector3(x, y, z));
                octomath::Vector3 target(tfTarget.getX(), tfTarget.getY(), tfTarget.getZ());
                octomath::Vector3 hit;
                // If
                if(hallucination.castRay(cam_pos, target, hit, true, r))
                {
                    hallucination.insertRay(cam_pos, hit, -1.0, true);
                }
                else
                {
                    hallucination.insertRay(cam_pos, target, -1.0, true);
                }
            }
        }
        hallucination.updateInnerOccupancy();

        // Check the roi occlusion in the hallucinated scene
        double new_occlusion = getBboxOccupancyInScene(hallucination, roi_min, roi_max).unknown;
        double gain = (old_occlusion - new_occlusion) / old_occlusion;

        // Send a marker
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.lifetime = ros::Duration();
        marker.scale.x = m_camera_constraints.range_min;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0 - gain;
        marker.color.g = gain;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.pose.position.x = position.getX();
        marker.pose.position.y = position.getY();
        marker.pose.position.z = position.getZ();
        tf::quaternionTFToMsg(orientation, marker.pose.orientation);
        marker.header.frame_id = map_msg.header.frame_id;
        marker.header.stamp = map_msg.header.stamp;
        static int markerid = 0;
        marker.id = markerid++;
        marker.ns = "asp_spatial_reasoner_debug";
        m_marker_pub.publish(marker);
    }


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
