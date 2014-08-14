#ifndef OCTREE_REGIONS_H
#define OCTREE_REGIONS_H

#include <octomap/math/Vector3.h>
#include <vector>

class OcTreeBbox
{
public:
    octomath::Vector3 min, max;

    OcTreeBbox(octomath::Vector3 const & min, octomath::Vector3 const & max) : min(min), max(max) {}

    bool contains(octomath::Vector3 const & p) const
    {
        return min.x() <= p.x() && min.y() <= p.y() && min.z() <= p.z() &&
               max.x() >= p.x() && max.y() >= p.y() && max.z() >= p.z();
    }
};

class OcTreeROI
{
public:
    std::vector<OcTreeBbox> elements;

    bool contains(octomath::Vector3 const & p) const
    {
        for(std::vector<OcTreeBbox>::const_iterator it = elements.begin(); it != elements.end(); ++it)
        {
            if(it->contains(p))
            {
                return true;
            }
        }
        return false;
    }
};

#endif // OCTREE_REGIONS_H

