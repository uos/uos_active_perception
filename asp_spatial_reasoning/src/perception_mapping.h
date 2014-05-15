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
    octomap::OcTree const & getFringeMap() const;
    void setResolution(double const & resolution);
    int countRevealableVoxels(octomap::point3d const & camera,
                              octomap::point3d const & fringe,
                              double const & max_range);
    std::vector<octomap::point3d> getFringeCenters(octomap::point3d min, octomap::point3d max);

private:
    octomap::OcTree m_occupancy_map, m_fringe_map;
};

#endif // PERCEPTIONMAPPING_H
