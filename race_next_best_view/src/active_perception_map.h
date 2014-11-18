#ifndef ACTIVE_PERCEPTION_MAP_H
#define ACTIVE_PERCEPTION_MAP_H

#include "camera_constraints.h"
#include "octree_regions.h"

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>


class ActivePerceptionMap
{
public:
    typedef boost::unordered_map<octomap::OcTreeKey, unsigned int, octomap::OcTreeKey::KeyHash> OcTreeKeyMap;
    typedef boost::unordered_set<unsigned int> ObjectIdSet;
    struct ObjectIdSetHash
    {
        std::size_t operator()(const ObjectIdSet & t) const
        {
              std::size_t val = 0;
              for(ObjectIdSet::iterator it = t.begin(); it != t.end(); ++it) {
                  val += *it;
              }
              return val;
        }
    };
    typedef boost::unordered_map<ObjectIdSet, unsigned int, ObjectIdSetHash> ObjectSetMap;

    ActivePerceptionMap(double const & resolution);

    void integratePointCloud(octomap::Pointcloud const & scan,
                             octomap::pose6d const & scan_pose,
                             tf::Pose const & camera_pose,
                             CameraConstraints const & camera_constraints);
    void resetVolume(octomap::point3d const & min, octomap::point3d const & max);
    void setOccupied(octomap::point3d const & min, octomap::point3d const & max);
    void updateInnerOccupancy();
    octomap::OcTree const & getOccupancyMap() const;
    octomap::OcTree const & getFringeMap() const;
    void setResolution(double const & resolution);
    void estimateRayGain(octomap::point3d const & camera,
                         octomap::point3d const & end,
                         OcTreeBoxSet const & roi,
                         octomap::KeySet & discovered_keys) const;
    void estimateRayGainObjectAware(octomap::point3d const & camera,
                                    octomap::point3d const & end,
                                    OcTreeBoxSet const & roi,
                                    OcTreeBoxSet const & object_boxes,
                                    ObjectSetMap & object_sets,
                                    OcTreeKeyMap & visibility_map) const;
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
