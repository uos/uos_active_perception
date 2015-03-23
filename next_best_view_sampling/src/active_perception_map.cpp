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

#include "active_perception_map.h"

#include "octree_ray_iterator.h"
#include "octree_regions.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

#include <stdexcept>

const std::string ActivePerceptionMap::F_OCCUPANCY_MAP = "occupancy_map.bt";
const std::string ActivePerceptionMap::F_FRINGE_MAP = "frontier_map.bt";

ActivePerceptionMap::ActivePerceptionMap(double const & resolution) :
    m_occupancy_map(resolution),
    m_fringe_map(resolution)
{
    m_occupancy_map.enableChangeDetection(true);
}

ActivePerceptionMap::ActivePerceptionMap(std::string const & prefix) :
    m_occupancy_map(0.01),
    m_fringe_map(0.01)
{
    if(m_occupancy_map.readBinary(prefix + F_OCCUPANCY_MAP) &&
       m_fringe_map.readBinary(prefix + F_FRINGE_MAP))
    {
        m_occupancy_map.enableChangeDetection(true);
    }
    else
    {
        throw std::runtime_error("Error loading octomap files: \n"
                                 + prefix + F_OCCUPANCY_MAP + "\n" + prefix + F_FRINGE_MAP);
    }
}

/** Integrates a point cloud in occupancy and fringe map.
  * scan_pose: The frame of origin for the point cloud
  * camera_pose: The frame of origin for the sensor according to ROS convention
  * The reason we need both is that scan_pose may be an optical_frame with x pointing right and y pointing down,
  * while we need camera_pose to follow ROS convention and have x pointing forward.
  */
void ActivePerceptionMap::integratePointCloud(octomap::Pointcloud const & scan,
                                              octomap::pose6d const & scan_pose,
                                              tf::Pose const & camera_pose,
                                              CameraConstraints const & camera_constraints)
{
    // update occupancy map
    octomap::KeySet touched_keys;
    for(octomap::Pointcloud::const_iterator it = scan.begin(); it != scan.end(); ++it)
    {
        double dist = it->norm();
        if(camera_constraints.range_min < dist && dist < camera_constraints.range_max)
        {
            octomap::point3d p = scan_pose.transform(*it);
            m_occupancy_map.updateNode(p, true, true);
            touched_keys.insert(m_occupancy_map.coordToKey(p));
        }
    }

    // Detect space that should have been seen but was not (exceeding sensor max range).
    // This space is assumed to be empty and updated in the occupancy map.
    double azimuth_min = -camera_constraints.hfov / 2.0;
    double azimuth_max =  camera_constraints.hfov / 2.0;
    double inclination_min = -camera_constraints.vfov / 2.0;
    double inclination_max =  camera_constraints.vfov / 2.0;
    double angle_increment = rayAngleStep(camera_constraints.range_max);
    octomath::Vector3 scan_origin = octomap::pointTfToOctomap(camera_pose.getOrigin());
    for(double azimuth = azimuth_min; azimuth <= azimuth_max; azimuth += angle_increment)
    {
        for(double inclination = inclination_min; inclination <= inclination_max; inclination += angle_increment)
        {
            tf::Vector3 ray_end_in_cam = CameraConstraints::sphericalToCartesian(
                                         camera_constraints.range_max, inclination, azimuth);
            octomath::Vector3 ray_dir = octomap::pointTfToOctomap(camera_pose(ray_end_in_cam)) - scan_origin;
            for(RayIterator ray(m_occupancy_map, scan_origin, ray_dir);
                ray.distanceFromOrigin() < camera_constraints.range_max;
                ray.next())
            {
                if(touched_keys.count(ray.getKey()))
                {
                    break;
                }
                else
                {
                    m_occupancy_map.updateNode(ray.getKey(), false, true);
                }
            }
        }
    }
    m_occupancy_map.updateInnerOccupancy();

    // update fringe voxels
    for(octomap::KeyBoolMap::const_iterator it = m_occupancy_map.changedKeysBegin();
        it != m_occupancy_map.changedKeysEnd();
        ++it)
    {
        octomap::OcTreeKey key = it->first;
        m_fringe_map.deleteNode(key);
        if(m_occupancy_map.isNodeOccupied(m_occupancy_map.search(key)))
        {
            // Updated node is occupied, check if neighbours lose fringe property
            for(unsigned int i = 0; i < 6; i++)
            {
                // Get neighbor key
                octomap::OcTreeKey candidate = key;
                candidate[i/2] += (i % 2 == 0) ? -1 : 1;
                if(m_fringe_map.search(candidate))
                {
                    // Check if this node still has fringe property
                    bool is_fringe = false;
                    for(unsigned int k = 0; k < 6; k++)
                    {
                        // Get neighbor key
                        octomap::OcTreeKey neighbor = candidate;
                        neighbor[k/2] += (k % 2 == 0) ? -1 : 1;
                        // Check if this is free in occupancy map
                        if(octomap::OcTreeNode * nptr = m_occupancy_map.search(neighbor))
                        {
                            if(!m_occupancy_map.isNodeOccupied(nptr))
                            {
                                is_fringe = true;
                                break;
                            }
                        }
                    }
                    if(!is_fringe) m_fringe_map.deleteNode(candidate);
                }
            }
        }
        else
        {
            // Updated node is free, check neighbor voxels for potential fringeness
            for(unsigned int i = 0; i < 6; i++)
            {
                // Get neighbor key
                octomap::OcTreeKey candidate = key;
                candidate[i/2] += (i % 2 == 0) ? -1 : 1;
                // Check if this is unknown in occupancy map
                if(!m_occupancy_map.search(candidate))
                {
                    m_fringe_map.updateNode(candidate, true);
                }
            }
        }
    }

    m_occupancy_map.resetChangeDetection();
}

/**
  Removes a bounding box from the map. Fringe voxels are updated accordingly.
  */
void ActivePerceptionMap::resetVolume(octomap::point3d const & min, octomap::point3d const & max, bool keep_occupied)
{
    deleteMapVolume(m_occupancy_map, min, max, !keep_occupied, true);
    deleteMapVolume(m_fringe_map, min, max, true, true);

    // check boundary voxels for fringeness
    std::vector<octomap::OcTreeKey> boundary = getBoundaryVoxels(min, max);
    for(std::vector<octomap::OcTreeKey>::iterator it = boundary.begin();
        it < boundary.end();
        ++it)
    {
        // Check if neighbors are known
        for(unsigned int i = 0; i < 6; i++)
        {
            // Get neighbor key
            octomap::OcTreeKey neighbor = *it;
            neighbor[i/2] += (i % 2 == 0) ? -1 : 1;
            // Check if this is known in occupancy map
            if(octomap::OcTreeNode * node_ptr = m_occupancy_map.search(neighbor))
            {
                if(!m_occupancy_map.isNodeOccupied(node_ptr))
                {
                     m_fringe_map.updateNode(*it, true);
                }
            }
        }
    }
}

/**
  Marks the whole given area as occupied. Caller must call updateInnerOccupancy when done.
  */
void ActivePerceptionMap::setOccupied(octomap::point3d const & min, octomap::point3d const & max)
{
    octomap::OcTreeKey min_key = m_occupancy_map.coordToKey(min);
    octomap::OcTreeKey max_key = m_occupancy_map.coordToKey(max);
    for(int x = min_key[0]; x <= max_key[0]; x++)
    {
        for(int y = min_key[1]; y <= max_key[1]; y++)
        {
            for(int z = min_key[2]; z <= max_key[2]; z++)
            {
                m_occupancy_map.updateNode(octomap::OcTreeKey(x,y,z), true, true);
                m_fringe_map.deleteNode(octomap::OcTreeKey(x,y,z));
            }
        }
    }
}

void ActivePerceptionMap::updateInnerOccupancy()
{
    m_occupancy_map.updateInnerOccupancy();
    m_fringe_map.updateInnerOccupancy();
}

octomap::OcTree const & ActivePerceptionMap::getOccupancyMap() const
{
    return m_occupancy_map;
}

octomap::OcTree const & ActivePerceptionMap::getFringeMap() const
{
    return m_fringe_map;
}

void ActivePerceptionMap::setResolution(double const & resolution)
{
    m_occupancy_map.setResolution(resolution);
    m_fringe_map.setResolution(resolution);
}

void ActivePerceptionMap::estimateRayGainObjectAware
(
        octomap::point3d const & camera,
        octomap::point3d const & end,
        OcTreeBoxSet const & roi,
        OcTreeBoxSet const & object_boxes,
        ObjectSetMap & object_sets,
        OcTreeKeyMap & visibility_map) const
{
    octomath::Vector3 direction = end - camera;
    double length = direction.norm();
    boost::unordered_set<unsigned int> condition;
    unsigned int condition_id;
    try {
        condition_id = object_sets.at(condition);
    } catch(std::out_of_range& e) {
        condition_id = object_sets.size();
        object_sets[condition] = condition_id;
    }

    for(RayIterator ray(m_occupancy_map, camera, direction); ray.distanceFromOrigin() < length; ray.next())
    {
        if(octomap::OcTreeNode * node_ptr = m_occupancy_map.search(ray.getKey()))
        {
            if(m_occupancy_map.isNodeOccupied(node_ptr))
            {
                unsigned int hit_obj = object_boxes.getContainingBoxId(ray.getKey());
                if(hit_obj)
                {
                    condition.insert(hit_obj - 1);
                    try {
                        condition_id = object_sets.at(condition);
                    } catch(std::out_of_range& e) {
                        condition_id = object_sets.size();
                        object_sets[condition] = condition_id;
                    }
                    continue;
                }
                else
                {
                    break;
                }
            }
        }
        else if(roi.elements.empty() || roi.getContainingBoxId(ray.getKey()))
        {
            visibility_map.insert(OcTreeKeyMap::value_type(ray.getKey(), condition_id));
            // It is possible that a cell can be seen with multiple rays that have different conditions
            // (on object corners). Since we simply overwrite conditions that maybe already exist, the outcome
            // of corner cases is undefined.
        }
    }
}

/** Returns a vector from the center of unknown cells in ROI towards the center of free cells. */
octomath::Vector3 ActivePerceptionMap::getFringeNormal(octomap::point3d const & p, OcTreeBoxSet const & roi)
{
    octomap::OcTreeKey key = m_occupancy_map.coordToKey(p);
    octomap::point3d roi_sum = p;
    octomap::point3d outside_sum(0.0, 0.0, 0.0);
    size_t roi_n = 1;
    size_t outside_n = 0;
    for(int i = -1; i <= 1; ++i) for(int j = -1; j <= 1; ++j) for(int k = -1; k <= 1; ++k)
    {
        octomap::OcTreeKey neighbor(key[0]+i, key[1]+j, key[2]+k);
        if(neighbor == key) continue;
        octomap::OcTreeNode * nnode = m_occupancy_map.search(neighbor);
        if(nnode)
        {
            if(!m_occupancy_map.isNodeOccupied(nnode))
            {
                outside_sum += m_occupancy_map.keyToCoord(neighbor);
                outside_n++;
            }
        }
        else
        {
            if(roi.getContainingBoxId(neighbor))
            {
                roi_sum += m_occupancy_map.keyToCoord(neighbor);
                roi_n++;
            }
            else
            {
                outside_sum += m_occupancy_map.keyToCoord(neighbor);
                outside_n++;
            }
        }
    }
    assert(outside_n > 0);
    return (outside_sum * (1.0 / outside_n)) - (roi_sum * (1.0 / roi_n));
}

std::vector<octomap::point3d> ActivePerceptionMap::getFringeCenters(octomap::point3d min, octomap::point3d max)
{
    std::vector<octomap::point3d> centers;
    for(octomap::OcTree::leaf_bbx_iterator it = m_fringe_map.begin_leafs_bbx(min,max);
        it != m_fringe_map.end_leafs_bbx();
        ++it)
    {
        centers.push_back(it.getCoordinate());
    }
    return centers;
}

std::vector<octomap::point3d> ActivePerceptionMap::getFringeCenters()
{
    std::vector<octomap::point3d> centers;
    for(octomap::OcTree::leaf_iterator it = m_fringe_map.begin_leafs();
        it != m_fringe_map.end_leafs();
        ++it)
    {
        centers.push_back(it.getCoordinate());
    }
    return centers;
}

/**
  If a bounding box lies within unknown space and therefore does not contain any fringe voxels in itself,
  we want to start observations at the box's faces. This method returns the centers of all voxels that
  - are part of the face of the bounding box and
  - are unknown and
  - are not already fringe voxels and
  - have at least one unknown neighbor outside of the boundary
  */
std::vector<octomap::point3d> ActivePerceptionMap::genBoundaryFringeCenters
(
        octomap::point3d const & min,
        octomap::point3d const & max) const
{
    std::vector<octomap::point3d> centers;
    std::vector<octomap::OcTreeKey> boundary = getBoundaryVoxels(min, max);
    OcTreeBbox box(m_occupancy_map.coordToKey(min), m_occupancy_map.coordToKey(max));
    for(std::vector<octomap::OcTreeKey>::iterator it = boundary.begin();
        it < boundary.end();
        ++it)
    {
        if(!m_fringe_map.search(*it) && !m_occupancy_map.search(*it))
        {
            // This is a candidate, check the neighbors
            for(unsigned int i = 0; i < 6; i++)
            {
                octomap::OcTreeKey neighbor = *it;
                neighbor[i/2] += (i % 2 == 0) ? -1 : 1;
                if(!box.contains(neighbor) && !m_occupancy_map.search(neighbor))
                {
                    centers.push_back(m_occupancy_map.keyToCoord(*it));
                    break;
                }
            }
        }
    }

    return centers;
}

/**
  Generates a map visualization as rviz marker.
  Caller is supposed to fill in the fields
  - header
  - ns
  - id
  */
visualization_msgs::Marker ActivePerceptionMap::genOccupancyMarker() const
{
    double r = m_occupancy_map.getResolution();
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = r;
    marker.scale.y = r;
    marker.scale.z = r;
    marker.color.a = 1.0;
    for(octomap::OcTree::leaf_iterator it = m_occupancy_map.begin_leafs();
        it != m_occupancy_map.end_leafs();
        it++)
    {
        if(m_occupancy_map.isNodeOccupied(*it))
        {
            double size_multiplier = it.getSize() / r;
            octomap::point3d first_coord = it.getCoordinate()
                                         - (octomap::point3d(1, 1, 1) * ((it.getSize() - r) / 2.0));
            for(int i = 0; i < size_multiplier; i++)
            {
                for(int j = 0; j < size_multiplier; j++)
                {
                    for(int k = 0; k < size_multiplier; k++)
                    {
                        octomap::point3d coord = first_coord + octomap::point3d(i*r, j*r, k*r);
                        std_msgs::ColorRGBA color;
                        double grey = 1.0 - ((coord.z()-0.1) / 1.9);
                        if(grey > 1.0) continue;//grey = 1.0;
                        if(grey < 0.0) continue;//grey = 0.0;
                        color.r = grey;
                        color.g = grey;
                        color.b = grey;
                        marker.points.push_back(octomap::pointOctomapToMsg(coord));
                        marker.colors.push_back(color);
                    }
                }
            }
        }
    }
    return marker;
}

/**
  Generates a map visualization as rviz marker.
  Caller is supposed to fill in the fields
  - header
  - ns
  - id
  */
visualization_msgs::Marker ActivePerceptionMap::genFringeMarker() const
{
    double r = m_occupancy_map.getResolution();
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = r;
    marker.scale.y = r;
    marker.scale.z = r;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;
    for(octomap::OcTree::leaf_iterator it = m_fringe_map.begin_leafs();
        it != m_fringe_map.end_leafs();
        it++)
    {
        if(m_fringe_map.isNodeOccupied(*it))
        {
            double size_multiplier = it.getSize() / r;
            octomap::point3d first_coord = it.getCoordinate()
                                         - (octomap::point3d(1, 1, 1) * ((it.getSize() - r) / 2.0));
            for(int i = 0; i < size_multiplier; i++)
            {
                for(int j = 0; j < size_multiplier; j++)
                {
                    for(int k = 0; k < size_multiplier; k++)
                    {
                        octomap::point3d coord = first_coord + octomap::point3d(i*r, j*r, k*r);
                        marker.points.push_back(octomap::pointOctomapToMsg(coord));
                    }
                }
            }
        }
    }
    return marker;
}

/**
  Returns a list of boundary voxels for a given volume
  */
std::vector<octomap::OcTreeKey> ActivePerceptionMap::getBoundaryVoxels
(
        octomap::point3d const & min,
        octomap::point3d const & max) const
{
    std::vector<octomap::OcTreeKey> boundary;
    octomap::OcTreeKey min_key = m_occupancy_map.coordToKey(min);
    octomap::OcTreeKey max_key = m_occupancy_map.coordToKey(max);

    for(int x = min_key[0]; x <= max_key[0]; x++)
    {
        for(int y = min_key[1]; y <= max_key[1]; y++)
        {
            // lower face
            boundary.push_back(octomap::OcTreeKey(x, y, min_key[2]));
            // upper face
            boundary.push_back(octomap::OcTreeKey(x, y, max_key[2]));
            // side faces
            if(x == min_key[0] || y == min_key[1] ||
               x == max_key[0] || y == max_key[1])
            {
                for(int z = min_key[2] + 1; z < max_key[2]; z++)
                {
                    boundary.push_back(octomap::OcTreeKey(x, y, z));
                }
            }
        }
    }

    return boundary;
}

/** Find the right discretization of ray angles so that each octree voxel at range is hit by one ray. */
double ActivePerceptionMap::rayAngleStep(const double & range) const
{
    return std::acos(1.0 - (std::pow(m_occupancy_map.getResolution(), 2) / (2.0 * std::pow(range, 2))));
}

void ActivePerceptionMap::deleteMapVolume
(
        octomap::OcTree & tree,
        octomap::point3d const & min,
        octomap::point3d const & max,
        bool delete_occupied,
        bool delete_free)
{
    octomap::OcTreeKey min_key = tree.coordToKey(min);
    octomap::OcTreeKey max_key = tree.coordToKey(max);
    for(int x = min_key[0]; x <= max_key[0]; x++)
    {
        for(int y = min_key[1]; y <= max_key[1]; y++)
        {
            for(int z = min_key[2]; z <= max_key[2]; z++)
            {
                octomap::OcTreeNode * node_ptr = tree.search(octomap::OcTreeKey(x, y, z));
                if(node_ptr)
                {
                    if((!tree.isNodeOccupied(node_ptr) && delete_free) ||
                       ( tree.isNodeOccupied(node_ptr) && delete_occupied))
                    {
                        tree.deleteNode(octomap::OcTreeKey(x, y, z));
                    }
                }
            }
        }
    }
}

bool ActivePerceptionMap::serializeToFiles(const std::string & prefix)
{
    m_occupancy_map.prune();
    m_fringe_map.prune();
    return m_occupancy_map.writeBinary(prefix + F_OCCUPANCY_MAP) && m_fringe_map.writeBinary(prefix + F_FRINGE_MAP);
}
