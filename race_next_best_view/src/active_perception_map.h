#ifndef ACTIVE_PERCEPTION_MAP_H
#define ACTIVE_PERCEPTION_MAP_H

#include "camera_constraints.h"
#include "octree_regions.h"

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

class ActivePerceptionMap
{
public:
    ActivePerceptionMap(double const & resolution);

    void integratePointCloud(octomap::Pointcloud const & scan,
                             octomap::pose6d const & scan_pose,
                             tf::Pose const & camera_pose,
                             CameraConstraints const & camera_constraints);
    void resetVolume(octomap::point3d const & min, octomap::point3d const & max);
    octomap::OcTree const & getOccupancyMap() const;
    octomap::OcTree const & getFringeMap() const;
    void setResolution(double const & resolution);
    double fringeSubmergence(octomap::point3d const & camera,
                             octomap::point3d const & fringe,
                             double const & max_range);
    double estimateRayGain(octomap::point3d const & camera,
                           octomap::point3d const & end,
                           OcTreeROI const & roi) const;
    std::vector<octomap::point3d> getFringeCenters(octomap::point3d min, octomap::point3d max);
    std::vector<octomap::point3d> getFringeCenters();
    std::vector<octomap::point3d> genBoundaryFringeCenters(octomap::point3d const & min,
                                                           octomap::point3d const & max) const;
    visualization_msgs::Marker genOccupancyMarker() const;
    visualization_msgs::Marker genFringeMarker() const;

private:
    octomap::OcTree m_occupancy_map, m_fringe_map;

    std::vector<octomap::OcTreeKey> getBoundaryVoxels(octomap::point3d const & min,
                                                      octomap::point3d const & max) const;

    static void deleteMapVolume(octomap::OcTree & tree,
                                octomap::point3d const & min,
                                octomap::point3d const & max);
};

#endif // ACTIVE_PERCEPTION_MAP_H
