/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Thorsten Gedicke
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include "next_best_view_node.h"

#include "active_perception_map.h"
#include "octree_regions.h"
#include "observation_pose_sampler.h"

#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/Marker.h>
#include <boost/random.hpp>
#include <uos_active_perception_msgs/ConditionalVisibilityMap.h>
#include <uos_active_perception_msgs/CellIds.h>

#include <ctime>
#include <list>
#include <memory>
#include <stdint.h>
#include <cstring>

#define PUB_FRINGE_NORMAL_MARKERS false

NextBestViewNode::NextBestViewNode() :
    m_node_handle("~"),
    m_node_handle_pub(),
    m_point_cloud_subscriber(m_node_handle_pub.subscribe(
                                 "cloud_in",
                                 1,
                                 &NextBestViewNode::pointCloudCb,
                                 this)),
    m_static_map_subscriber(),
    m_marker_translator_subscriber(m_node_handle.subscribe(
                                       "marker_in",
                                       1,
                                       &NextBestViewNode::markerTranslatorCb,
                                       this)),
    m_get_bbox_percent_unseen_server(m_node_handle_pub.advertiseService(
                                         "/get_bbox_occupancy",
                                         &NextBestViewNode::getBboxOccupancyCb,
                                         this)),
    m_get_observation_camera_poses_server(m_node_handle_pub.advertiseService(
                                              "/get_observation_camera_poses",
                                              &NextBestViewNode::getObservationCameraPosesCb,
                                              this)),
    m_evaluate_observation_camera_poses_server(m_node_handle_pub.advertiseService(
                                                   "/evaluate_observation_camera_poses",
                                                   &NextBestViewNode::evaluateObservationCameraPosesCb,
                                                   this)),
    m_reset_volumes_server(m_node_handle_pub.advertiseService(
                                       "/reset_volumes",
                                       &NextBestViewNode::resetVolumesCb,
                                       this)),
    m_write_map_server(m_node_handle_pub.advertiseService(
                                       "/write_map",
                                       &NextBestViewNode::saveMapCb,
                                       this)),
    m_load_map_server(m_node_handle_pub.advertiseService(
                                       "/load_map",
                                       &NextBestViewNode::loadMapCb,
                                       this)),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/next_best_view_marker", 20000))
{
    m_node_handle.param("resolution"    , m_resolution    , 0.05);
    m_node_handle.param("camera_range_tolerance", m_camera_range_tolerance, 0.01);
    m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));

    // Set camera constraints from parameters (Defaults: xtion on calvin)
    m_node_handle.param("camera_frame_id",m_camera_constraints.frame_id  , std::string("/head_mount_kinect"));
    m_node_handle.param("height_min"    , m_camera_constraints.height_min, 1.5875);
    m_node_handle.param("height_max"    , m_camera_constraints.height_max, 1.5875);
    m_node_handle.param("pitch_min"     , m_camera_constraints.pitch_min , -0.935815); // [-53.6182 deg]
    m_node_handle.param("pitch_max"     , m_camera_constraints.pitch_max , -0.935815); // [-53.6182 deg]
    m_node_handle.param("range_min"     , m_camera_constraints.range_min , 0.4);
    m_node_handle.param("range_max"     , m_camera_constraints.range_max , 3.0);
    m_node_handle.param("hfov"          , m_camera_constraints.hfov      , 1.01229097);
    m_node_handle.param("vfov"          , m_camera_constraints.vfov      , 0.785398163);
    m_node_handle.param("roll"          , m_camera_constraints.roll      , PI);

    std::string initial_map_prefix;
    m_node_handle.param("initial_map", initial_map_prefix, std::string(""));
    if(!initial_map_prefix.empty())
    {
        m_perception_map.reset(new ActivePerceptionMap(initial_map_prefix));
        m_resolution = m_perception_map->getOccupancyMap().getResolution();
    }
    else
    {
        m_perception_map.reset(new ActivePerceptionMap(m_resolution));
    }

    bool load_2dmap;
    m_node_handle.param("load_2dmap", load_2dmap, false);
    if(load_2dmap)
    {
        m_static_map_subscriber = m_node_handle_pub.subscribe("/map", 1, &NextBestViewNode::staticMapCb, this);
    }
}

double NextBestViewNode::getIntersectionVolume(
        const octomath::Vector3 &box1min,
        const octomath::Vector3 &box1max,
        const octomath::Vector3 &box2min,
        const octomath::Vector3 &box2max
){
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
    return (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
}

std::vector<tf::Vector3> NextBestViewNode::bboxVertices(uos_active_perception_msgs::BoundingBox const & bbox)
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

bool NextBestViewNode::getAxisAlignedBounds(
        uos_active_perception_msgs::BoundingBox const & bbox,
        octomath::Vector3 & min,
        octomath::Vector3 & max
) const
{
    float inf = std::numeric_limits<float>::infinity();
    min.x() =  inf; min.y() =  inf; min.z() =  inf;
    max.x() = -inf; max.y() = -inf; max.z() = -inf;

    std::vector<tf::Vector3> bbox_vertices = bboxVertices(bbox);
    if(!m_tf_listener.waitForTransform(m_world_frame_id,
                                       bbox.pose_stamped.header.frame_id,
                                       bbox.pose_stamped.header.stamp,
                                       ros::Duration(1)))
    {
        ROS_ERROR_STREAM("next_best_view_node: Timed out while waiting for transform from " <<
                          bbox.pose_stamped.header.frame_id << " to " <<
                          m_world_frame_id);
        return false;
    }
    for(std::vector<tf::Vector3>::iterator it = bbox_vertices.begin(); it != bbox_vertices.end(); ++it)
    {
        geometry_msgs::PointStamped pin, pout;
        pin.header = bbox.pose_stamped.header;
        pin.point.x = it->x();
        pin.point.y = it->y();
        pin.point.z = it->z();
        m_tf_listener.transformPoint(m_world_frame_id, pin, pout);
        if(pout.point.x < min.x()) min.x() = pout.point.x;
        if(pout.point.y < min.y()) min.y() = pout.point.y;
        if(pout.point.z < min.z()) min.z() = pout.point.z;
        if(pout.point.x > max.x()) max.x() = pout.point.x;
        if(pout.point.y > max.y()) max.y() = pout.point.y;
        if(pout.point.z > max.z()) max.z() = pout.point.z;
    }
    return true;
}

OcTreeBoxSet NextBestViewNode::boxSetFromMsg(std::vector<uos_active_perception_msgs::BoundingBox> const & bbox_vec) const
{
    OcTreeBoxSet boxSet;
    for(unsigned int i = 0; i < bbox_vec.size(); i++)
    {
        octomath::Vector3 min, max;
        if(getAxisAlignedBounds(bbox_vec[i], min, max))
        {
            octomap::OcTreeKey mink, maxk;
            if(m_perception_map->getOccupancyMap().coordToKeyChecked(min, mink) &&
               m_perception_map->getOccupancyMap().coordToKeyChecked(max, maxk))
            {
                boxSet.elements.push_back(OcTreeBbox(mink, maxk));
            }
        }
    }
    return boxSet;
}

std::vector<octomap::point3d> NextBestViewNode::getActiveFringe(
        const OcTreeBoxSet & roi,
        ActivePerceptionMap & map)
{
    std::vector<octomap::point3d> active_fringe;
    if(roi.elements.empty())
    {
        // If the requested roi is empty, use all fringe voxels
        active_fringe = map.getFringeCenters();
    }
    for(unsigned int i_roi = 0; i_roi < roi.elements.size(); i_roi++)
    {
        OcTreeBbox box = roi.elements[i_roi];
        octomath::Vector3 min = map.getFringeMap().keyToCoord(box.min);
        octomath::Vector3 max = map.getFringeMap().keyToCoord(box.max);
        std::vector<octomap::point3d> roi_fringe_centers;
        // add fringe voxels within the roi
        roi_fringe_centers = map.getFringeCenters(min, max);
        active_fringe.insert(active_fringe.end(), roi_fringe_centers.begin(), roi_fringe_centers.end());
        // generate fringe voxels at roi boundary
        roi_fringe_centers = map.genBoundaryFringeCenters(min, max);
        active_fringe.insert(active_fringe.end(), roi_fringe_centers.begin(), roi_fringe_centers.end());
    }
    // filter unobservable cells

    return active_fringe;
}

void NextBestViewNode::pubActiveFringe(const std::vector<octomap::point3d> & active_fringe,
                                       const std::vector<octomap::point3d> & active_fringe_normals)
{
    for(size_t i = 0; i < active_fringe_normals.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "active_fringe_normals";
        marker.id = i;
        marker.header.frame_id = m_world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.lifetime = ros::Duration(5.0);
        if(active_fringe_normals[i].norm() < std::numeric_limits<double>::epsilon())
        {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = m_resolution;
            marker.scale.y = m_resolution;
            marker.scale.z = m_resolution;
            marker.color.r = 1.0;
            marker.color.a = 0.75;
            marker.pose.position = octomap::pointOctomapToMsg(active_fringe[i]);
        }
        else
        {
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.01;
            marker.scale.y = 0.03;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.75;
            marker.points.push_back(octomap::pointOctomapToMsg(active_fringe[i]));
            marker.points.push_back(octomap::pointOctomapToMsg(active_fringe[i] +
                                                               active_fringe_normals[i].normalized() * 0.25));
        }
        m_marker_pub.publish(marker);
    }

    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = m_resolution;
    marker.scale.y = m_resolution;
    marker.scale.z = m_resolution;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    for(unsigned int i = 0; i < active_fringe.size(); i++)
    {
        marker.points.push_back(octomap::pointOctomapToMsg(active_fringe[i]));
    }
    marker.ns = "active_fringe";
    marker.id = 0;
    marker.header.frame_id = m_world_frame_id;
    marker.header.stamp = ros::Time::now();
    m_marker_pub.publish(marker);
}

bool NextBestViewNode::getBboxOccupancyCb(uos_active_perception_msgs::GetBboxOccupancy::Request &req,
                                          uos_active_perception_msgs::GetBboxOccupancy::Response &res)
{
    boost::mutex::scoped_lock lock(m_map_mutex);
    octomath::Vector3 min, max;
    getAxisAlignedBounds(req.bbox, min, max);
    octomap::OcTreeKey min_key = m_perception_map->getOccupancyMap().coordToKey(min);
    res.bbox_min.x = min_key[0];
    res.bbox_min.y = min_key[1];
    res.bbox_min.z = min_key[2];
    octomap::OcTreeKey max_key = m_perception_map->getOccupancyMap().coordToKey(max);
    res.bbox_max.x = max_key[0];
    res.bbox_max.y = max_key[1];
    res.bbox_max.z = max_key[2];

    for(int x = min_key[0]; x <= max_key[0]; x++)
    {
        for(int y = min_key[1]; y <= max_key[1]; y++)
        {
            for(int z = min_key[2]; z <= max_key[2]; z++)
            {
                octomap::OcTreeKey key(x,y,z);
                uos_active_perception_msgs::CellId cellid;
                cellid.x = x;
                cellid.y = y;
                cellid.z = z;
                if(octomap::OcTreeNode* node_ptr = m_perception_map->getOccupancyMap().search(key))
                {
                    if(m_perception_map->getOccupancyMap().isNodeOccupied(node_ptr))
                    {
                        res.occupied.cell_ids.push_back(cellid);
                    }
                    else
                    {
                        res.free.cell_ids.push_back(cellid);
                    }
                }
                else
                {
                    res.unknown.cell_ids.push_back(cellid);
                }
            }
        }
    }
    res.cell_volume = std::pow(m_resolution, 3);
    return true;
}

uos_active_perception_msgs::EvaluateObservationCameraPoses::Response NextBestViewNode::evaluateObservationCameraPoses
(
    const ActivePerceptionMap & map,
    const std::vector<tf::Pose> & observation_poses,
    const OcTreeBoxSet & roi,
    const OcTreeBoxSet & object_boxes,
    const double ray_skip,
    const ros::Duration timeout,
    const bool omit_cvm,
    const bool keep_blind_poses
) const {
    uos_active_perception_msgs::EvaluateObservationCameraPoses::Response res;
    ros::Time callback_timeout = ros::Time::now() + timeout;
    boost::mt19937 rng(std::time(0));
    boost::uniform_01<> rand_u01;
    ActivePerceptionMap::ObjectSetMap object_sets;
    object_sets.rehash(object_boxes.elements.size() / object_sets.max_load_factor());
    std::vector<visualization_msgs::Marker> markers;
    markers.reserve(observation_poses.size());
    size_t max_gain = 0;

    ROS_INFO_STREAM("NBV evaluation with " << object_boxes.elements.size() << " objects.");

    // Some derived camera parameters
    double azimuth_min = -m_camera_constraints.hfov / 2.0;
    double azimuth_max =  m_camera_constraints.hfov / 2.0;
    double inclination_min = -m_camera_constraints.vfov / 2.0;
    double inclination_max =  m_camera_constraints.vfov / 2.0;
    double angle_increment = map.rayAngleStep(m_camera_constraints.range_max);

    // fill roi_cell_counts of answer
    unsigned int roi_cell_count = roi.cellCount();
    if(!roi_cell_count)
    {
        // If the roi is empty (exploration mode), we estimate the number of cells in the view cone
        roi_cell_count =
        ((std::sqrt((2.0 * std::pow(m_camera_constraints.range_max, 2)) * (1.0 - std::cos(m_camera_constraints.hfov))) *
          std::sqrt((2.0 * std::pow(m_camera_constraints.range_max, 2)) * (1.0 - std::cos(m_camera_constraints.vfov))) *
          m_camera_constraints.range_max) / 3.0) / std::pow(m_resolution, 3);
    }
    res.roi_cell_counts.resize(roi.elements.size());
    for(size_t i = 0; i < roi.elements.size(); ++i)
    {
        res.roi_cell_counts[i] = roi.elements[i].cellCount();
    }

    for(size_t sample_id = 0; sample_id < observation_poses.size(); ++sample_id)
    {
        // check timeout
        if(timeout.toNSec() > 0 && ros::Time::now() > callback_timeout)
        {
            ROS_INFO_STREAM("Sampling timeout reached after n=" << sample_id);
            break;
        }

        const tf::Pose & sample = observation_poses[sample_id];
        const octomap::point3d cam_point = octomap::pointTfToOctomap(sample.getOrigin());

        ActivePerceptionMap::OcTreeKeyMap discovery_field;
        discovery_field.rehash(roi_cell_count / discovery_field.max_load_factor()); // reserve enough space

        for(double azimuth = azimuth_min; azimuth <= azimuth_max; azimuth += angle_increment)
        {
            for(double inclination = inclination_min; inclination <= inclination_max; inclination += angle_increment)
            {
                if(rand_u01(rng) < ray_skip)
                {
                    continue;
                }
                tf::Vector3 ray_end_in_cam = CameraConstraints::sphericalToCartesian(
                                             m_camera_constraints.range_max, inclination, azimuth);
                octomap::point3d ray_end = octomap::pointTfToOctomap(sample(ray_end_in_cam));
                map.estimateRayGainObjectAware(cam_point, ray_end, roi, object_boxes, object_sets, discovery_field);
            }
        }
        unsigned int gain = discovery_field.size();

        // Fetch target point
        octomap::point3d target_point_octomap;
        map.getOccupancyMap().castRay(cam_point,
                                      octomap::pointTfToOctomap(sample(tf::Vector3(1.0, 0.0, 0.0))) - cam_point,
                                      target_point_octomap,
                                      true,
                                      m_camera_constraints.range_max);

        // Write pose candidate to answer
        geometry_msgs::Pose camera_pose_msg;
        tf::poseTFToMsg(sample, camera_pose_msg);
        if(gain > 0 || keep_blind_poses)
        {
            res.camera_poses.push_back(camera_pose_msg);
            res.target_points.push_back(octomap::pointOctomapToMsg(target_point_octomap));
            res.information_gains.push_back(gain * std::pow(m_resolution, 3));
            if(!omit_cvm)
            {
                // Prepare the conditional visibility map for this sample
                boost::unordered_map<unsigned int, std::list<octomap::OcTreeKey> > cvm_hashed;
                for(ActivePerceptionMap::OcTreeKeyMap::iterator it = discovery_field.begin();
                    it != discovery_field.end();
                    ++it)
                {
                    cvm_hashed[it->second].push_back(it->first);
                }
                // Now build a cvm_msg from the hashed cvm
                // TODO: These types and conversion functions should probably get their own header
                uos_active_perception_msgs::ConditionalVisibilityMap cvm_msg;
                for(boost::unordered_map<unsigned int, std::list<octomap::OcTreeKey> >::iterator it = cvm_hashed.begin();
                    it != cvm_hashed.end();
                    ++it)
                {
                    cvm_msg.object_set_ids.push_back(it->first);
                    uos_active_perception_msgs::CellIds cell_ids_msg;
                    cell_ids_msg.cell_ids.reserve(it->second.size());
                    for(std::list<octomap::OcTreeKey>::iterator key_it = it->second.begin();
                        key_it != it->second.end();
                        ++key_it)
                    {
                        uos_active_perception_msgs::CellId cell_id_msg;
                        cell_id_msg.x = (*key_it)[0];
                        cell_id_msg.y = (*key_it)[1];
                        cell_id_msg.z = (*key_it)[2];
                        cell_ids_msg.cell_ids.push_back(cell_id_msg);
                    }
                    cvm_msg.cell_id_sets.push_back(cell_ids_msg);
                }
                res.cvms.push_back(cvm_msg);
            }
        }

        // Send a marker
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.lifetime = ros::Duration(10);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.2;
        if(gain > 0) {
            marker.color.g = 0.5;
            marker.color.r = 0.5;
            marker.color.b = 0.5;
        } else {
            marker.color.b = 1.0;
        }
        marker.color.a = 0.5;
        marker.pose = camera_pose_msg;
        marker.header.frame_id = m_world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id = sample_id;
        marker.ns = "nbv_samples";
        m_marker_pub.publish(marker);

        // save marker for later color-coding
        if(gain > 0) {
            marker.color.g = (double) gain;
            markers.push_back(marker);
        }
        if(gain > max_gain) {
            max_gain = gain;
        }
    }

    // color-code and re-publish markers
    for(size_t i = 0; i < markers.size(); ++i) {
        markers[i].color.g /= (double) max_gain;
        markers[i].color.r = 1.0 - markers[i].color.g;
        markers[i].color.b = 0.0;
        m_marker_pub.publish(markers[i]);
    }

    // Write object set ids
    res.object_sets.resize(object_sets.size());
    for(ActivePerceptionMap::ObjectSetMap::iterator it = object_sets.begin(); it != object_sets.end(); ++it)
    {
        res.object_sets[it->second].objects.reserve(it->first.size());
        res.object_sets[it->second].objects.insert(
                    res.object_sets[it->second].objects.begin(), it->first.begin(), it->first.end());
    }

    return res;
}

bool NextBestViewNode::getObservationCameraPosesCb(uos_active_perception_msgs::GetObservationCameraPoses::Request& req,
                                                   uos_active_perception_msgs::GetObservationCameraPoses::Response& res)
{
    ObservationPoseSampler ops(m_camera_constraints, m_camera_range_tolerance);
    boost::mt19937 rng(std::time(0));
    // TODO: Shrink the ROI to an area that is in principle observable
    // (not higher/lower than the camera constraints allow observations to be made)
    // Should be implemented together with the ObservationSampler class which will wrap the
    // sampleObservationSpace method and allow to ask for single samples

    // translate roi to octomap
    OcTreeBoxSet roi = boxSetFromMsg(req.roi);

    // make local copy of the map, so that sampling and mapping can occur in parallel
    std::auto_ptr<ActivePerceptionMap> map;
    {
        boost::mutex::scoped_lock lock(m_map_mutex);
        map.reset(new ActivePerceptionMap(*m_perception_map));
    }

    // Gather fringe voxel centers
    std::vector<octomap::point3d> active_fringe = getActiveFringe(roi, *map);
    std::vector<octomap::point3d> active_fringe_normals;
    #if(PUB_FRINGE_NORMAL_MARKERS)
    for(size_t i = 0; i < active_fringe.size(); ++i)
    {
        active_fringe_normals.push_back(map->getFringeNormal(active_fringe[i], roi));
    }
    #endif
    if(!roi.elements.empty())
    {
        pubActiveFringe(active_fringe, active_fringe_normals);
    }
    last_roi = roi;
    if(active_fringe.empty()) return true;

    bool observation_position_set = !req.observation_position.header.frame_id.empty();
    octomath::Vector3 observation_position;
    if(observation_position_set)
    {
        if(!m_tf_listener.waitForTransform(m_world_frame_id,
                                           req.observation_position.header.frame_id,
                                           req.observation_position.header.stamp,
                                           ros::Duration(1)))
        {
            ROS_ERROR_STREAM("next_best_view_node: Timed out while waiting for transform from " <<
                              req.observation_position.header.frame_id << " to " <<
                              m_world_frame_id);
            return false;
        }
        geometry_msgs::PointStamped obspos_msg;
        m_tf_listener.transformPoint(m_world_frame_id, req.observation_position, obspos_msg);
        observation_position = octomap::pointMsgToOctomap(obspos_msg.point);
    }

    std::vector<tf::Pose> samples;
    samples.reserve(req.sample_size);
    for(int sample_id = 0; !active_fringe.empty() && sample_id < req.sample_size; ++sample_id)
    {
        if(observation_position_set)
        {
            samples.push_back(ops.genPoseSample(observation_position, req.lock_height));
        }
        else
        {
            size_t fringeid = boost::uniform_int<>(0, active_fringe.size() - 1)(rng);
            octomap::point3d & poi = active_fringe[fringeid];
            octomath::Vector3 normal = map->getFringeNormal(poi, roi);
            try
            {
                samples.push_back(ops.genObservationSample(poi, normal));
            }
            catch(ObservationPoseSampler::UnobservableError& e)
            {
                active_fringe.erase(active_fringe.begin() + fringeid);
                --sample_id;
            }
            catch(ObservationPoseSampler::SamplingError& e)
            {
                {
                    visualization_msgs::Marker marker;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.ns = "sampling_errors";
                    marker.id = sample_id;
                    marker.header.frame_id = m_world_frame_id;
                    marker.lifetime = ros::Duration(5.0);
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = m_resolution;
                    marker.scale.y = m_resolution;
                    marker.scale.z = m_resolution;
                    marker.color.r = 1.0;
                    marker.color.a = 1.0;
                    marker.pose.position = octomap::pointOctomapToMsg(poi);
                    m_marker_pub.publish(marker);
                }{
                    visualization_msgs::Marker marker;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.ns = "sampling_errors";
                    marker.header.frame_id = m_world_frame_id;
                    marker.lifetime = ros::Duration(5.0);
                    marker.id = sample_id + req.sample_size;
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.scale.x = 0.01;
                    marker.scale.y = 0.03;
                    marker.color.r = 1.0;
                    marker.color.a = 1.0;
                    marker.points.push_back(octomap::pointOctomapToMsg(poi));
                    marker.points.push_back(octomap::pointOctomapToMsg(poi + normal.normalized() * 0.25));
                    m_marker_pub.publish(marker);
                }
                --sample_id;
            }
        }
    }

    uos_active_perception_msgs::EvaluateObservationCameraPoses::Response eval_res = evaluateObservationCameraPoses(
                *map,
                samples,
                boxSetFromMsg(req.roi),
                boxSetFromMsg(req.objects),
                req.ray_skip,
                req.timeout,
                req.omit_cvm,
                false);

    // Fill response fields
    res.camera_poses = eval_res.camera_poses;
    res.cvms = eval_res.cvms;
    res.information_gains = eval_res.information_gains;
    res.object_sets = eval_res.object_sets;
    res.roi_cell_counts = eval_res.roi_cell_counts;
    res.target_points = eval_res.target_points;

    return true;
}

bool NextBestViewNode::evaluateObservationCameraPosesCb
(
    uos_active_perception_msgs::EvaluateObservationCameraPoses::Request& req,
    uos_active_perception_msgs::EvaluateObservationCameraPoses::Response& res)
{
    std::vector<tf::Pose> cam_poses(req.camera_poses.size());
    for(size_t i = 0; i < req.camera_poses.size(); ++i)
    {
        tf::poseMsgToTF(req.camera_poses[i], cam_poses[i]);
    }
    // make local copy of the map, so that sampling and mapping can occur in parallel
    std::auto_ptr<ActivePerceptionMap> map;
    {
        boost::mutex::scoped_lock lock(m_map_mutex);
        map.reset(new ActivePerceptionMap(*m_perception_map));
    }
    OcTreeBoxSet roi = boxSetFromMsg(req.roi);
    // Do this stuff so the active fringe markers are updated
    std::vector<octomap::point3d> active_fringe = getActiveFringe(roi, *map);
    std::vector<octomap::point3d> active_fringe_normals;
    #if(PUB_FRINGE_NORMAL_MARKERS)
    for(size_t i = 0; i < active_fringe.size(); ++i)
    {
        active_fringe_normals.push_back(map->getFringeNormal(active_fringe[i], roi));
    }
    #endif
    if(!roi.elements.empty())
    {
        pubActiveFringe(active_fringe, active_fringe_normals);
    }
    last_roi = roi;

    res = evaluateObservationCameraPoses(*map,
                                         cam_poses,
                                         roi,
                                         boxSetFromMsg(req.objects),
                                         req.ray_skip,
                                         ros::Duration(),
                                         req.omit_cvm,
                                         true);
    return true;
}

void NextBestViewNode::pointCloudCb(sensor_msgs::PointCloud2 const & cloud)
{
    // Find sensor origin
    tf::StampedTransform sensor_to_world_tf, camera_to_world_tf;
    try
    {
        m_tf_listener.waitForTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, ros::Duration(2.0));
        m_tf_listener.lookupTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, sensor_to_world_tf);
        m_tf_listener.lookupTransform(m_world_frame_id, m_camera_constraints.frame_id, cloud.header.stamp, camera_to_world_tf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("next_best_view_node: Transform error of sensor data: "
                         << ex.what()
                         << ", cannot integrate data.");
        return;
    }

    // Ugly converter cascade to get octomap_cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    octomap::Pointcloud octomap_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    octomap::pointcloudPCLToOctomap(pcl_cloud, octomap_cloud);

    // Integrate point cloud
    boost::mutex::scoped_lock lock(m_map_mutex);
    m_perception_map->integratePointCloud(octomap_cloud,
                                          octomap::poseTfToOctomap(sensor_to_world_tf),
                                          camera_to_world_tf,
                                          m_camera_constraints);

    // Publish rviz map visualization
    {
        visualization_msgs::Marker marker = m_perception_map->genOccupancyMarker();
        marker.header.frame_id = m_world_frame_id;
        marker.id = 0;
        marker.ns = "occupancy_map";
        m_marker_pub.publish(marker);
        marker = m_perception_map->genFringeMarker();
        marker.header.frame_id = m_world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.ns = "fringe_map";
        m_marker_pub.publish(marker);
    }

    // Publish updated active fringe
    if(!last_roi.elements.empty())
    {
        std::vector<octomap::point3d> active_fringe = getActiveFringe(last_roi, *m_perception_map);
        std::vector<octomap::point3d> active_fringe_normals;
        #if(PUB_FRINGE_NORMAL_MARKERS)
        for(size_t i = 0; i < active_fringe.size(); ++i)
        {
            active_fringe_normals.push_back(m_perception_map.getFringeNormal(active_fringe[i], last_roi));
        }
        #endif
        pubActiveFringe(active_fringe, active_fringe_normals);
    }

    // Publish camera constraint marker
    {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = 0.005;
        marker.header.frame_id = m_camera_constraints.frame_id;
        marker.id = 0;
        marker.ns = "camera_constraints";
        double lr = m_camera_constraints.hfov / 2.0;
        double td = m_camera_constraints.vfov / 2.0;
        geometry_msgs::Point p;
        marker.points.push_back(geometry_msgs::Point());
        tf::pointTFToMsg(CameraConstraints::sphericalToCartesian(m_camera_constraints.range_max,  td,  lr), p);
        marker.points.push_back(p);
        marker.points.push_back(geometry_msgs::Point());
        tf::pointTFToMsg(CameraConstraints::sphericalToCartesian(m_camera_constraints.range_max, -td,  lr), p);
        marker.points.push_back(p);
        marker.points.push_back(geometry_msgs::Point());
        tf::pointTFToMsg(CameraConstraints::sphericalToCartesian(m_camera_constraints.range_max,  td, -lr), p);
        marker.points.push_back(p);
        marker.points.push_back(geometry_msgs::Point());
        tf::pointTFToMsg(CameraConstraints::sphericalToCartesian(m_camera_constraints.range_max, -td, -lr), p);
        marker.points.push_back(p);
        m_marker_pub.publish(marker);
    }
}

void NextBestViewNode::staticMapCb(nav_msgs::OccupancyGrid const & map)
{
    boost::mutex::scoped_lock lock(m_map_mutex);

    // Find map origin
    tf::StampedTransform map_to_world_tf;
    try
    {
        m_tf_listener.waitForTransform(m_world_frame_id, map.header.frame_id, map.header.stamp, ros::Duration(60.0));
        m_tf_listener.lookupTransform(m_world_frame_id, map.header.frame_id, map.header.stamp, map_to_world_tf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("next_best_view_node: Transform error of map origin: "
                         << ex.what()
                         << ", failed to integrate map data.");
        return;
    }

    tf::Transform map_tf;
    tf::poseMsgToTF(map.info.origin, map_tf);

    for(size_t w = 0; w < map.info.width; ++w) {
        for(size_t h = 0; h < map.info.height; ++h) {
            int8_t val = map.data[map.info.width * h + w];
            if(val > 65) {
                tf::Point p1, p2;
                p1.setX(w * map.info.resolution);
                p1.setY(h * map.info.resolution);
                p1.setZ(0.0);
                p2.setX((w + 1) * map.info.resolution);
                p2.setY((h + 1) * map.info.resolution);
                p2.setZ(0.0);

                p1 = map_to_world_tf(map_tf(p1));
                p2 = map_to_world_tf(map_tf(p2));

                // Set walls to a height of 2.5 m.
                p1.setZ(0.0);
                p2.setZ(2.5);

                m_perception_map->setOccupied(octomap::pointTfToOctomap(p1), octomap::pointTfToOctomap(p2));
            }
        }
    }

    // Mark floor as occupied
    {
        tf::Point p1, p2;
        p1.setX(0.0);
        p1.setY(0.0);
        p1.setZ(0.0);
        p2.setX((map.info.width + 1) * map.info.resolution);
        p2.setY((map.info.height + 1) * map.info.resolution);
        p2.setZ(0.0);

        p1 = map_to_world_tf(map_tf(p1));
        p2 = map_to_world_tf(map_tf(p2));

        // Set walls to a height of 3 m.
        p1.setZ(0.0);
        p2.setZ(0.0);

        m_perception_map->setOccupied(octomap::pointTfToOctomap(p1), octomap::pointTfToOctomap(p2));
    }

    m_perception_map->updateInnerOccupancy();

    ROS_INFO("next_best_view_node: Added 2d map walls to octomap");
}

void NextBestViewNode::markerTranslatorCb(visualization_msgs::Marker marker)
{
    marker.header.frame_id = m_world_frame_id;
    marker.scale.x = m_resolution;
    marker.scale.y = m_resolution;
    marker.scale.z = m_resolution;
    boost::mutex::scoped_lock lock(m_map_mutex);
    for(size_t i = 0; i < marker.points.size(); ++i)
    {
        octomap::point3d p = m_perception_map->getOccupancyMap().keyToCoord(
                             octomap::OcTreeKey(marker.points[i].x, marker.points[i].y, marker.points[i].z));
        marker.points[i].x = p.x();
        marker.points[i].y = p.y();
        marker.points[i].z = p.z();
    }
    m_marker_pub.publish(marker);
}

bool NextBestViewNode::resetVolumesCb
(
        uos_active_perception_msgs::ResetVolumes::Request & req,
        uos_active_perception_msgs::ResetVolumes::Response & resp)
{
    boost::mutex::scoped_lock lock(m_map_mutex);
    bool success = true;
    for(std::vector<uos_active_perception_msgs::BoundingBox>::iterator it = req.volumes.begin();
        it < req.volumes.end();
        ++it)
    {
        octomath::Vector3 min, max;
        if(getAxisAlignedBounds(*it, min, max))
        {
            m_perception_map->resetVolume(min, max, req.keep_occupied);
        }
        else
        {
            success = false;
        }
    }
    return success;
}

bool NextBestViewNode::saveMapCb
(
    uos_active_perception_msgs::ReadWriteFiles::Request& req,
    uos_active_perception_msgs::ReadWriteFiles::Response& resp
){
    boost::mutex::scoped_lock lock(m_map_mutex);
    return m_perception_map->serializeToFiles(req.prefix);
}

bool NextBestViewNode::loadMapCb
(
    uos_active_perception_msgs::ReadWriteFiles::Request& req,
    uos_active_perception_msgs::ReadWriteFiles::Response& resp
){
    boost::mutex::scoped_lock lock(m_map_mutex);
    m_perception_map.reset(new ActivePerceptionMap(req.prefix));
    m_resolution = m_perception_map->getOccupancyMap().getResolution();
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "next_best_view_node");
    NextBestViewNode node;
    ROS_INFO("next_best_view_node: Initialized!");
    ros::MultiThreadedSpinner(4).spin();
}
