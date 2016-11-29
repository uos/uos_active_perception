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

#include "ros/ros.h"
#include "uos_active_perception_msgs/GetObservationCameraPoses.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "uos_active_perception_msgs/GetBboxOccupancy.h"
#include "uos_active_perception_msgs/ResetVolumes.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_object_search_servicetest");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("servicetest_marker", 1);

    // Wait for subscribers to connect
    ros::Duration(1).sleep();

    // Create the request
    uos_active_perception_msgs::BoundingBox box;
    box.pose_stamped.header.frame_id = "/map";
    box.pose_stamped.header.stamp = ros::Time::now();
    box.pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    nh_private.param("x", box.pose_stamped.pose.position.x, 5.2);
    nh_private.param("y", box.pose_stamped.pose.position.y, 2.0);
    nh_private.param("z", box.pose_stamped.pose.position.z, 0.3);
    nh_private.param("xdim", box.dimensions.x, 1.0);
    nh_private.param("ydim", box.dimensions.y, 2.0);
    nh_private.param("zdim", box.dimensions.z, 0.6);

    // Send a marker
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration();
    marker.scale.x = box.dimensions.x;
    marker.scale.y = box.dimensions.y;
    marker.scale.z = box.dimensions.z;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.pose = box.pose_stamped.pose;
    marker.header = box.pose_stamped.header;
    marker.id = 7;
    marker.ns = "servicetest";
    marker_pub.publish(marker);

    // Send request
    uos_active_perception_msgs::GetBboxOccupancy get_bbox;
    get_bbox.request.bbox = box;
    if(ros::service::call("get_bbox_occupancy", get_bbox))
    {
        ROS_INFO_STREAM("service call get_bbox_occupancy successful: " << get_bbox.response.free.cell_ids.size() << " free / " << get_bbox.response.occupied.cell_ids.size() << " occupied / " << get_bbox.response.unknown.cell_ids.size() << " unknown");
    }
    else
    {
        ROS_ERROR("service call failed!");
    }

    // Send request
    uos_active_perception_msgs::GetObservationCameraPoses get_ocp;
    get_ocp.request.roi.push_back(box);
    get_ocp.request.sample_size = 100;
    get_ocp.request.ray_skip = 0.8;
    if(ros::service::call("get_observation_camera_poses", get_ocp))
    {
        ROS_INFO_STREAM("service call get_observation_camera_poses successful: " << get_ocp.response.camera_poses.size() << " camera poses.");
    }
    else
    {
        ROS_ERROR("service call failed!");
    }
    
    // Send request
    uos_active_perception_msgs::ResetVolumes reset_volumes_call;
    reset_volumes_call.request.volumes.push_back(box);
    if(ros::service::call("reset_volumes", reset_volumes_call))
    {
        ROS_INFO_STREAM("service call reset_volumes successful.");
    }
    else
    {
        ROS_ERROR("service call failed!");
    }

    ros::spinOnce();
}

