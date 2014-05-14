#ifndef PERCEPTIONMAPPING_H
#define PERCEPTIONMAPPING_H

#include "octomap/OcTree.h"
#include "octomap/Pointcloud.h"

class PerceptionMapping
{
public:
    PerceptionMapping(double const & resolution);
    void integratePointCloud(octomap::Pointcloud const & scan,
                             octomap::pose6d const & frame_origin);
    octomap::OcTree const & getOccupancyMap() const;
    void setResolution(double const & resolution);

private:
    octomap::OcTree m_occupancy_map, m_fringe_map;
};

#endif // PERCEPTIONMAPPING_H
