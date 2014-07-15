#include "active_perception_map.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

ActivePerceptionMap::ActivePerceptionMap(double const & resolution) :
    m_occupancy_map(resolution),
    m_fringe_map(resolution)
{
    m_occupancy_map.enableChangeDetection(true);
}

void ActivePerceptionMap::integratePointCloud(octomap::Pointcloud const & scan,
                                              octomap::pose6d const & frame_origin)
{
    m_occupancy_map.insertPointCloud(scan, octomap::point3d(0, 0, 0), frame_origin);
    // Update fringe map
    for(octomap::KeyBoolMap::const_iterator it = m_occupancy_map.changedKeysBegin();
        it != m_occupancy_map.changedKeysEnd();
        ++it)
    {
        // Only do something if the observed node is free
        if(!m_occupancy_map.isNodeOccupied(m_occupancy_map.search(it->first)))
        {
            octomap::OcTreeKey key = it->first;
            m_fringe_map.deleteNode(key);
            // Check neighbor voxels for fringeiness
            for(unsigned int i = 0; i < 6; i++)
            {
                // Get neighbor key
                octomap::OcTreeKey neighbor = key;
                neighbor[i/2] += (i % 2 == 0) ? -1 : 1;
                // Check if this is unknown in occupancy map
                if(!m_occupancy_map.search(neighbor))
                {
                    m_fringe_map.updateNode(neighbor, true);
                }
            }
        }
    }
    m_occupancy_map.resetChangeDetection();
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

double ActivePerceptionMap::fringeSubmergence(octomap::point3d const & camera,
                                              octomap::point3d const & fringe,
                                              double const & max_range)
{
    octomath::Vector3 ray_dir, ray_end;
    // Find out where the ray ends
    ray_dir = fringe - camera;
    if(!m_occupancy_map.castRay(camera, ray_dir, ray_end, true, ray_dir.norm()))
    {
        return max_range - camera.distance(fringe);
    }
    else
    {
        return 0.0;
    }
}

/**
  Traces a ray and returns the sum over all segments of the ray that cross unknown space AFTER seeing free space
  at least once before.
  */
double ActivePerceptionMap::estimateRayGain(octomap::point3d const & camera, octomap::point3d const & end) const
{
    octomap::KeyRay ray;
    m_occupancy_map.computeRayKeys(camera, end, ray);
    // TODO: computeRayKeys for the whole ray is extremely slow. Instead, the implementation of castRay should be
    //       adapted for this method so that ray traversal is terminated early.
    double gain = 0.0;
    bool traversing_free = false, gaining = false;
    octomap::point3d gain_onset;
    for(octomap::KeyRay::iterator key_it = ray.begin(); key_it != ray.end(); ++key_it)
    {
        if(octomap::OcTreeNode* node_ptr = m_occupancy_map.search(*key_it))
        {
            if(!m_occupancy_map.isNodeOccupied(node_ptr))
            {
                if(gaining)
                {
                    gain += gain_onset.distance(m_occupancy_map.keyToCoord(*key_it));
                    gaining = false;
                }
                traversing_free = true;
            }
            else
            {
                break;
            }
        }
        else if(traversing_free)
        {
            traversing_free = false;
            gaining = true;
            gain_onset = m_occupancy_map.keyToCoord(*key_it);
        }
    }
    if(gaining)
    {
        gain += gain_onset.distance(end);
    }
    return gain;
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
  Generates a map visualization as rviz marker.
  Caller is supposed to fill in the fields
  - header
  - ns
  - id
  */
visualization_msgs::Marker ActivePerceptionMap::genOccupancyMarker() const
{
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = m_occupancy_map.getResolution();
    marker.scale.y = m_occupancy_map.getResolution();
    marker.scale.z = m_occupancy_map.getResolution();
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    for(octomap::OcTree::leaf_iterator it = m_occupancy_map.begin_leafs();
        it != m_occupancy_map.end_leafs();
        it++)
    {
        if(m_occupancy_map.isNodeOccupied(*it))
        {
            marker.points.push_back(octomap::pointOctomapToMsg(it.getCoordinate()));
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
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.lifetime = ros::Duration();
    marker.scale.x = m_fringe_map.getResolution();
    marker.scale.y = m_fringe_map.getResolution();
    marker.scale.z = m_fringe_map.getResolution();
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
            marker.points.push_back(octomap::pointOctomapToMsg(it.getCoordinate()));
        }
    }
    return marker;
}
