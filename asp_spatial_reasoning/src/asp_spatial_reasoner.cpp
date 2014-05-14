#include "asp_spatial_reasoner.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asp_spatial_reasoning/GetBboxOccupancy.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "octomap_msgs/GetOctomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "octomap_ros/conversions.h"
#include "octomap/math/Vector3.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
#include "random_numbers/random_numbers.h"
#include "perception_mapping.h"

#include <sstream>
#include <vector>
#include <list>
#include <limits>
#include <algorithm>
#include <cmath>

AspSpatialReasoner::AspSpatialReasoner() :
    m_node_handle("~"),
    m_node_handle_pub(),
    m_point_cloud_subscriber(m_node_handle_pub.subscribe(
                                 "cloud_in",
                                 1,
                                 &AspSpatialReasoner::pointCloudCb,
                                 this)),
    m_get_bbox_percent_unseen_server(m_node_handle_pub.advertiseService(
                                         "/get_bbox_occupancy",
                                         &AspSpatialReasoner::getBboxOccupancyCb,
                                         this)),
    m_get_observation_camera_poses_server(m_node_handle_pub.advertiseService(
                                              "/get_observation_camera_poses",
                                              &AspSpatialReasoner::getObservationCameraPosesCb,
                                              this)),
    m_get_objects_to_remove_server(m_node_handle_pub.advertiseService(
                                       "/get_objects_to_remove",
                                       &AspSpatialReasoner::getObjectsToRemoveCb,
                                       this)),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/visualization_marker", 10000)),
    m_occupancy_octree_pub(m_node_handle_pub.advertise<octomap_msgs::Octomap>("occupancy_octree", 1)),
    m_camera_constraints(),
    m_perception_mapping(0.01)
{
    // Set camera constraints from parameters (Defaults: Kinect on [insert robot])
    m_node_handle.param("height_min"    , m_camera_constraints.height_min, 0.5);
    m_node_handle.param("height_max"    , m_camera_constraints.height_max, 1.5);
    m_node_handle.param("pitch_min"     , m_camera_constraints.pitch_min , -0.174532925);
    m_node_handle.param("pitch_max"     , m_camera_constraints.pitch_max , 0.174532925);
    m_node_handle.param("range_min"     , m_camera_constraints.range_min , 0.4);
    m_node_handle.param("range_max"     , m_camera_constraints.range_max , 5.0);
    m_node_handle.param("hfov"          , m_camera_constraints.hfov      , 1.01229097);
    m_node_handle.param("vfov"          , m_camera_constraints.vfov      , 0.767944871);
    m_node_handle.param("resolution"    , m_resolution                   , 0.1); // [m]
    m_node_handle.param("sample_size"   , m_sample_size                  , 500);
    m_node_handle.param("world_frame_id", m_world_frame_id               , std::string("/odom_combined"));

    m_perception_mapping.setResolution(m_resolution);
}

octomap_msgs::Octomap AspSpatialReasoner::getCurrentScene() const
{
    octomap_msgs::Octomap msg;
    octomap_msgs::binaryMapToMsg(m_perception_mapping.getOccupancyMap(), msg);
    msg.header.frame_id = m_world_frame_id;
    msg.header.stamp = ros::Time::now();
    msg.resolution = m_resolution;
    return msg;
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

double AspSpatialReasoner::getInformationGainForRegionRemoval(octomap::OcTree const & octree,
                                          std::list<octomath::Vector3> const & unknown_voxels,
                                          octomath::Vector3 const & remove_min,
                                          octomath::Vector3 const & remove_max,
                                          octomath::Vector3 const & cam_position) const
{
    // Create a hallucinated octree with the object removed
    octomap::OcTree hallucination(octree);
    for(octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(remove_min, remove_max);
        it != octree.end();
        ++it)
    {
        hallucination.deleteNode(it.getKey(), it.getDepth());
        // TODO: It is either getKey() or getIndexKey(). Don't know the difference, the documentation is bad.
    }

    // Cast rays to unknown voxels and count revealed ones.
    long n_revealed_voxels = 0;
    for(std::list<octomath::Vector3>::const_iterator it = unknown_voxels.begin();
        it != unknown_voxels.end();
        ++it)
    {
        octomath::Vector3 target_dir = *it - cam_position;
        double target_distance = target_dir.norm();
        // Check target voxel visibility
        octomath::Vector3 ray_hit;
        if(target_distance < m_camera_constraints.range_max && // in range
           !hallucination.castRay(cam_position, target_dir, ray_hit, true, target_distance)) // not occluded
        {
            n_revealed_voxels++;
        }
    }

    return static_cast<double>(unknown_voxels.size()) / static_cast<double>(n_revealed_voxels);
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
    // TODO: Find out if this is necessary
    octree->toMaxLikelihood();
    octree->prune();

    // Gather unknown voxel centers
    octomath::Vector3 roi_min, roi_max;
    if(!getAxisAlignedBounds(map_msg.header.frame_id, req.roi, roi_min, roi_max))
    {
        return false;
    }

    std::list<octomath::Vector3> unknown_voxels;
    octree->getUnknownLeafCenters(unknown_voxels, roi_min, roi_max);
    std::vector<octomath::Vector3> unknown_voxels_initial(unknown_voxels.begin(), unknown_voxels.end());

    double azimuth_min = -m_camera_constraints.hfov / 2.0;
    double azimuth_max =  m_camera_constraints.hfov / 2.0;
    double inclination_min = -m_camera_constraints.vfov / 2.0;
    double inclination_max =  m_camera_constraints.vfov / 2.0;

    random_numbers::RandomNumberGenerator rng;
    double max_range_increment = m_camera_constraints.range_max - m_camera_constraints.range_min;
    for(int i = 0; i < m_sample_size; ++i)
    {
        // Pick a random unknown voxel as center
        octomath::Vector3& center_voxel = unknown_voxels_initial.at(
                    rng.uniformInteger(0, unknown_voxels_initial.size() - 1));
        // Create random position within feasible range and height
        double direction = rng.uniform01() * 2.0 * PI;
        double distance = (octree->getResolution() / 2.0) + m_camera_constraints.range_min
                                                          + rng.uniform01() * max_range_increment;
        double x_offset = distance * std::cos(direction);
        double y_offset = distance * std::sin(direction);
        double max_z_offset = std::sqrt(  (m_camera_constraints.range_max * m_camera_constraints.range_max)
                                        - (distance * distance));
        double z_min = std::max(center_voxel.z() - max_z_offset, m_camera_constraints.height_min);
        double z_max = std::min(center_voxel.z() + max_z_offset, m_camera_constraints.height_max);
        tf::Vector3 position(center_voxel.x() + x_offset,
                             center_voxel.y() + y_offset,
                             z_min + rng.uniform01() * (z_max - z_min));
        // TODO: Above calculations will go wrong if height constraints are not in octomap frame

        // Point the created pose towards the target voxel
        tf::Vector3 forward_axis(1, 0, 0);
        tf::Vector3 center_direction(center_voxel.x() - position.getX(),
                                     center_voxel.y() - position.getY(),
                                     center_voxel.z() - position.getZ());

        tf::Quaternion orientation(tf::tfCross(forward_axis, center_direction),
                                   tf::tfAngle(forward_axis, center_direction));
        tf::Transform camera_tf(orientation, position);
        tf::Transform camera_tf_inverse = camera_tf.inverse();

        octomath::Vector3 position_om(position.getX(), position.getY(), position.getZ());

        // Filter infeasible pitch
        double cam_pitch =  0.5 * PI - tf::tfAngle(tf::Vector3(0, 0, 1), center_direction);
        if(cam_pitch < m_camera_constraints.pitch_min || cam_pitch > m_camera_constraints.pitch_max)
        {
            continue;
        }

        // Cast rays at unknown voxels and count observable ones
        long n_revealed_voxels = 0;
        for(std::list<octomath::Vector3>::iterator it = unknown_voxels.begin();
            it != unknown_voxels.end();
            ++it)
        {
            octomath::Vector3 target_dir = *it - position_om;
            double target_distance = target_dir.norm();
            // Find ray angles
            tf::Vector3 target_in_cam(it->x(), it->y(), it->z());
            target_in_cam = camera_tf_inverse(target_in_cam);
            double ray_azimuth     = std::atan2(target_in_cam.y(), target_in_cam.x());
            double ray_inclination = std::atan2(target_in_cam.z(), target_in_cam.x());
            octomath::Vector3 ray_hit;

//            // Send a marker
//            visualization_msgs::Marker marker;
//            marker.action = visualization_msgs::Marker::ADD;
//            marker.type = visualization_msgs::Marker::ARROW;
//            marker.lifetime = ros::Duration();
//            marker.scale.x = 0.01;
//            marker.scale.y = 0.01;
//            //marker.scale.z = 0.0;
//            marker.color.a = 0.5;
//            marker.header = map_msg.header;
//            static int markerid = 0;
//            marker.id = markerid++;
//            marker.ns = "asp_spatial_reasoner_debug";
//            geometry_msgs::Point p;
//            tf::pointTFToMsg(camera_tf.getOrigin(), p);
//            marker.points.push_back(p);
//            geometry_msgs::Point p2;
//            p2.x = it->x();
//            p2.y = it->y();
//            p2.z = it->z();
//            marker.points.push_back(p2);

            // Check target voxel visibility
            if(target_distance < m_camera_constraints.range_max && // in range
               ray_azimuth > azimuth_min && ray_azimuth < azimuth_max && // in hfov
               ray_inclination > inclination_min && ray_inclination < inclination_max && // in vfov
               !octree->castRay(position_om, target_dir, ray_hit, true, target_distance)) // not occluded
            {
                // Voxel is revealed
                n_revealed_voxels++;

//                marker.color.r = 0.0;
//                marker.color.g = 1.0;
//                marker.color.b = 0.0;
            }
//            else
//            {
//                marker.color.r = 1.0;
//                marker.color.g = 0.0;
//                marker.color.b = 0.0;
//            }
//            m_marker_pub.publish(marker);
        }

        double gain = static_cast<double>(n_revealed_voxels) /
                      static_cast<double>(unknown_voxels_initial.size());

        // Write pose candidate to answer
        geometry_msgs::PoseStamped camera_pose_msg;
        tf::poseTFToMsg(camera_tf, camera_pose_msg.pose);
        camera_pose_msg.header.frame_id = map_msg.header.frame_id;
        camera_pose_msg.header.stamp = map_msg.header.stamp;
        if(n_revealed_voxels > 0)
        {
            res.camera_poses.push_back(camera_pose_msg);
            res.information_gain.push_back(gain);
        }


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
        marker.pose = camera_pose_msg.pose;
        marker.header = camera_pose_msg.header;
        static int markerid = 0;
        marker.id = markerid++;
        marker.ns = "asp_spatial_reasoner_debug";
        m_marker_pub.publish(marker);
    }
    delete octree;
    return true;
}

bool AspSpatialReasoner::getObjectsToRemoveCb(asp_spatial_reasoning::GetObjectsToRemove::Request& req,
                                              asp_spatial_reasoning::GetObjectsToRemove::Response& res)
{
    // Retrieve octomap of current scene
    octomap_msgs::Octomap map_msg = getCurrentScene();
    if(map_msg.data.size() == 0)
    {
        ROS_ERROR("asp_spatial_reasoner: Could not retrieve current octomap");
        return false;
    }
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map_msg));
    // TODO: Find out if this is necessary
    octree->toMaxLikelihood();
    octree->prune();

    // Gather unknown voxel centers
    octomath::Vector3 roi_min, roi_max;
    if(!getAxisAlignedBounds(map_msg.header.frame_id, req.roi, roi_min, roi_max))
    {
        return false;
    }

    std::list<octomath::Vector3> unknown_voxels;
    octree->getUnknownLeafCenters(unknown_voxels, roi_min, roi_max);

    // TODO: Use this method on all removable objects in the scene
    // getInformationGainForRegionRemoval(*octree, unknown_voxels, remove_min, remove_max, cam_position);

    return true;
}

void AspSpatialReasoner::pointCloudCb(sensor_msgs::PointCloud2 const & cloud)
{
    // Find sensor origin
    tf::StampedTransform sensor_to_world_tf;
    try
    {
        m_tf_listener.waitForTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, ros::Duration(2.0));
        m_tf_listener.lookupTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, sensor_to_world_tf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("asp_spatial_reasoner: Transform error of sensor data: "
                         << ex.what()
                         << ", cannot integrate data.");
        return;
    }

    // Ugly converter cascade to get octomap_cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    octomap::Pointcloud octomap_cloud;
    // ****************************************************************************************************************
    // * WARNING! THE FOLLOWING LINE DESTROYS THE MESSAGE!
    // * This is fine as long as this code always runs in its own process. Should this ever be run with shared memory,
    // * this is unsafe and should be replaced by:
    // * pcl::fromROSMsg(cloud, pcl_cloud);
    // ****************************************************************************************************************
    pcl::moveFromROSMsg(const_cast<sensor_msgs::PointCloud2&>(cloud), pcl_cloud);
    octomap::pointcloudPCLToOctomap(pcl_cloud, octomap_cloud);

    m_perception_mapping.integratePointCloud(octomap_cloud, octomap::poseTfToOctomap(sensor_to_world_tf));

    m_occupancy_octree_pub.publish(getCurrentScene());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "asp_spatial_reasoner");
    AspSpatialReasoner node;
    ROS_INFO("asp_spatial_reasoner: Initialized!");
    ros::spin();
}
