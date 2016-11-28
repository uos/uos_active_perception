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

#ifndef NEXT_BEST_VIEW_NODE_H
#define NEXT_BEST_VIEW_NODE_H

#include "active_perception_map.h"
#include "camera_constraints.h"
#include "octree_regions.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <uos_active_perception_msgs/GetBboxOccupancy.h>
#include <uos_active_perception_msgs/GetObservationCameraPoses.h>
#include <uos_active_perception_msgs/EvaluateObservationCameraPoses.h>
#include <uos_active_perception_msgs/ResetVolumes.h>
#include <uos_active_perception_msgs/BoundingBox.h>
#include <uos_active_perception_msgs/ReadWriteFiles.h>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class NextBestViewNode
{
public:
    NextBestViewNode();

private:
    static const double PI = 3.14159265359;

    ros::NodeHandle m_node_handle, m_node_handle_pub;
    ros::Subscriber m_point_cloud_subscriber;
    ros::Subscriber m_static_map_subscriber;
    ros::Subscriber m_marker_translator_subscriber;
    ros::ServiceServer m_get_bbox_percent_unseen_server;
    ros::ServiceServer m_get_observation_camera_poses_server;
    ros::ServiceServer m_evaluate_observation_camera_poses_server;
    ros::ServiceServer m_reset_volumes_server;
    ros::ServiceServer m_write_map_server;
    ros::ServiceServer m_load_map_server;
    tf::TransformListener m_tf_listener;
    ros::Publisher m_marker_pub;
    CameraConstraints m_camera_constraints;
    std::auto_ptr<ActivePerceptionMap> m_perception_map;
    double m_resolution, m_camera_range_tolerance;
    std::string m_world_frame_id;
    boost::mutex m_map_mutex;
    OcTreeBoxSet last_roi;

    static double getIntersectionVolume(const octomath::Vector3 &box1min, const octomath::Vector3 &box1max,
                                        const octomath::Vector3 &box2min, const octomath::Vector3 &box2max);
    static std::vector<tf::Vector3> bboxVertices(uos_active_perception_msgs::BoundingBox const & bbox);

    bool getAxisAlignedBounds(uos_active_perception_msgs::BoundingBox const & bbox,
                              octomath::Vector3 & min,
                              octomath::Vector3 & max) const;

    OcTreeBoxSet boxSetFromMsg(std::vector<uos_active_perception_msgs::BoundingBox> const & bbox_vec) const;

    std::vector<octomap::point3d> getActiveFringe(const OcTreeBoxSet & roi,  ActivePerceptionMap & map);
    void pubActiveFringe(const std::vector<octomap::point3d> & active_fringe,
                         const std::vector<octomap::point3d> & active_fringe_normals);

    uos_active_perception_msgs::EvaluateObservationCameraPoses::Response evaluateObservationCameraPoses
    (
        const ActivePerceptionMap & map,
        const std::vector<tf::Pose> & observation_poses,
        const OcTreeBoxSet & roi,
        const OcTreeBoxSet & object_boxes,
        const double ray_skip,
        const ros::Duration timeout,
        const bool omit_cvm,
        const bool keep_blind_poses
    ) const;

    // Callbacks
    void pointCloudCb(const PointCloud::ConstPtr &cloud);

    void staticMapCb(nav_msgs::OccupancyGrid const & map);

    void markerTranslatorCb(visualization_msgs::Marker marker);

    bool getBboxOccupancyCb(uos_active_perception_msgs::GetBboxOccupancy::Request&,
                            uos_active_perception_msgs::GetBboxOccupancy::Response&);

    bool getObservationCameraPosesCb(uos_active_perception_msgs::GetObservationCameraPoses::Request&,
                                     uos_active_perception_msgs::GetObservationCameraPoses::Response&);

    bool evaluateObservationCameraPosesCb(uos_active_perception_msgs::EvaluateObservationCameraPoses::Request&,
                                          uos_active_perception_msgs::EvaluateObservationCameraPoses::Response&);

    bool resetVolumesCb(uos_active_perception_msgs::ResetVolumes::Request&,
                        uos_active_perception_msgs::ResetVolumes::Response&);

    bool saveMapCb(uos_active_perception_msgs::ReadWriteFiles::Request&,
                   uos_active_perception_msgs::ReadWriteFiles::Response&);

    bool loadMapCb(uos_active_perception_msgs::ReadWriteFiles::Request&,
                   uos_active_perception_msgs::ReadWriteFiles::Response&);
};

#endif // NEXT_BEST_VIEW_NODE_H
