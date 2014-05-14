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
    for(octomap::KeyBoolMap::const_iterator it = m_occupancy_map.changedKeysBegin();
        it != m_occupancy_map.changedKeysEnd();
        ++it)
    {
        // TODO: update fringe map
    }
    m_occupancy_map.resetChangeDetection();
}

octomap::OcTree const & PerceptionMapping::getOccupancyMap() const
{
    return m_occupancy_map;
}

void PerceptionMapping::setResolution(double const & resolution)
{
    m_occupancy_map.setResolution(resolution);
    m_fringe_map.setResolution(resolution);
}
