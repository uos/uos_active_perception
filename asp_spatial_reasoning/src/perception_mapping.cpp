#include "perception_mapping.h"

#include "octomap/octomap.h"
#include "octomap_ros/conversions.h"

PerceptionMapping::PerceptionMapping(double const & resolution) :
    m_occupancy_map(resolution),
    m_fringe_map(resolution)
{
    m_occupancy_map.enableChangeDetection(true);
}

void PerceptionMapping::integratePointCloud(octomap::Pointcloud const & scan,
                                       octomap::pose6d const & frame_origin)
{
    m_occupancy_map.insertPointCloud(scan, octomap::point3d(0, 0, 0), frame_origin);
    // Update fringe map
    for(octomap::KeyBoolMap::const_iterator it = m_occupancy_map.changedKeysBegin();
        it != m_occupancy_map.changedKeysEnd();
        ++it)
    {
        // Only do something if the observed node is free
        if(m_occupancy_map.search(it->first)->getOccupancy() < 0.5)
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

octomap::OcTree const & PerceptionMapping::getOccupancyMap() const
{
    return m_occupancy_map;
}

octomap::OcTree const & PerceptionMapping::getFringeMap() const
{
    return m_fringe_map;
}

void PerceptionMapping::setResolution(double const & resolution)
{
    m_occupancy_map.setResolution(resolution);
    m_fringe_map.setResolution(resolution);
}

/**
  Computes the amount of unknown voxels visible through a fringe voxel.
  */
int PerceptionMapping::countRevealableVoxels(octomap::point3d const & camera,
                                             octomap::point3d const & fringe,
                                             double const & max_range)
{
    octomath::Vector3 ray_dir, ray_end;
    // Find out where the ray ends
    ray_dir = fringe - camera;
    m_occupancy_map.castRay(camera, ray_dir, ray_end, true, max_range);
    if(camera.distance(ray_end) < camera.distance(fringe))
    {
        // Fringe could not be observed
        return 0;
    }
    // Observe what happens after the ray enters the fringe
    octomap::KeyRay ray;
    m_occupancy_map.computeRayKeys(fringe, ray_end, ray);
    int unknown_voxels = 0;
    for(octomap::KeyRay::iterator it = ray.begin(); it != ray.end(); ++it)
    {
        if(!m_occupancy_map.search(*it))
        {
            unknown_voxels++;
        }
    }
    return unknown_voxels;
}

std::vector<octomap::point3d> PerceptionMapping::getFringeCenters(octomap::point3d min, octomap::point3d max)
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

std::vector<octomap::point3d> PerceptionMapping::getFringeCenters()
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
