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
    void resetVolume(octomap::point3d const & min, octomap::point3d const & max, bool keep_occupied);
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
    octomath::Vector3 getFringeNormal(octomap::point3d const & p, OcTreeBoxSet const & roi);
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
                                octomap::point3d const & max,
                                bool delete_occupied,
                                bool delete_free);
};

#endif // ACTIVE_PERCEPTION_MAP_H
