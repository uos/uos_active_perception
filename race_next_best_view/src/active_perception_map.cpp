#include "active_perception_map.h"

#include "octree_ray_iterator.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

ActivePerceptionMap::ActivePerceptionMap(double const & resolution) :
    m_occupancy_map(resolution),
    m_fringe_map(resolution)
{
    m_occupancy_map.enableChangeDetection(true);
}

void ActivePerceptionMap::integratePointCloud(octomap::Pointcloud const & scan,
                                              octomap::pose6d const & scan_pose,
                                              tf::Pose const & camera_pose,
                                              CameraConstraints const & camera_constraints)
{
    // update occupancy map
    m_occupancy_map.insertPointCloud(scan, octomap::point3d(0, 0, 0), scan_pose);

    // Detect space that should have been seen but was not (exceeding sensor max range).
    // This space is assumed to be empty and updated in the occupancy map.
    double azimuth_min = -camera_constraints.hfov / 2.0;
    double azimuth_max =  camera_constraints.hfov / 2.0;
    double inclination_min = -camera_constraints.vfov / 2.0;
    double inclination_max =  camera_constraints.vfov / 2.0;
    // Find the right discretization of ray angles so that each octree voxel at max range is hit by one ray.
    double angle_increment =
            std::acos(1.0 - (std::pow(m_occupancy_map.getResolution(), 2) /
                             (2.0 * std::pow(camera_constraints.range_max, 2))));
    octomath::Vector3 scan_origin = scan_pose.trans();
    for(double azimuth = azimuth_min; azimuth <= azimuth_max; azimuth += angle_increment)
    {
        for(double inclination = inclination_min; inclination <= inclination_max; inclination += angle_increment)
        {
            tf::Vector3 ray_end_in_cam(camera_constraints.range_max * std::cos(azimuth),
                                       camera_constraints.range_max * std::sin(azimuth),
                                       camera_constraints.range_max * std::sin(inclination));
            octomath::Vector3 ray_dir = octomap::pointTfToOctomap(camera_pose(ray_end_in_cam)) - scan_origin;
            for(RayIterator ray(m_occupancy_map, scan_origin, ray_dir);
                ray.distanceFromOrigin() < camera_constraints.range_max;
                ray.next())
            {
                if(octomap::OcTreeNode* node_ptr = m_occupancy_map.search(ray.getKey()))
                {
                    if(m_occupancy_map.isNodeOccupied(node_ptr))
                    {
                        break;
                    }
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
        // Only add to fringe if the observed node is free
        if(!m_occupancy_map.isNodeOccupied(m_occupancy_map.search(key)))
        {
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

/**
  Removes a bounding box from the map. Fringe voxels are updated accordingly.
  */
void ActivePerceptionMap::resetVolume(octomap::point3d const & min, octomap::point3d const & max)
{
    deleteMapVolume(m_occupancy_map, min, max);
    deleteMapVolume(m_fringe_map, min, max);

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

void ActivePerceptionMap::estimateRayGain
(
        octomap::point3d const & camera,
        octomap::point3d const & end,
        OcTreeROI const & roi,
        octomap::KeySet & discovered_keys) const
{
    octomath::Vector3 direction = end - camera;
    double length = direction.norm();

    for(RayIterator ray(m_occupancy_map, camera, direction); ray.distanceFromOrigin() < length; ray.next())
    {
        if(octomap::OcTreeNode * node_ptr = m_occupancy_map.search(ray.getKey()))
        {
            if(m_occupancy_map.isNodeOccupied(node_ptr))
            {
                break;
            }
        }
        else if(roi.elements.empty() || roi.contains(ray.getKey()))
        {
            discovered_keys.insert(ray.getKey());
        }
    }
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
  - are not already fringe voxels
  */
std::vector<octomap::point3d> ActivePerceptionMap::genBoundaryFringeCenters
(
        octomap::point3d const & min,
        octomap::point3d const & max) const
{
    std::vector<octomap::point3d> centers;
    std::vector<octomap::OcTreeKey> boundary = getBoundaryVoxels(min, max);
    for(std::vector<octomap::OcTreeKey>::iterator it = boundary.begin();
        it < boundary.end();
        ++it)
    {
        if(!m_fringe_map.search(*it) && !m_occupancy_map.search(*it))
        {
            centers.push_back(m_occupancy_map.keyToCoord(*it));
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

void ActivePerceptionMap::deleteMapVolume
(
        octomap::OcTree & tree,
        octomap::point3d const & min,
        octomap::point3d const & max)
{
    octomap::OcTreeKey min_key = tree.coordToKey(min);
    octomap::OcTreeKey max_key = tree.coordToKey(max);
    for(int x = min_key[0]; x <= max_key[0]; x++)
    {
        for(int y = min_key[1]; y <= max_key[1]; y++)
        {
            for(int z = min_key[2]; z <= max_key[2]; z++)
            {
                tree.deleteNode(octomap::OcTreeKey(x, y, z));
            }
        }
    }
}
