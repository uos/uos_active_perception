#ifndef OCTREE_REGIONS_H
#define OCTREE_REGIONS_H

#include <vector>
#include <octomap/OcTreeKey.h>

class OcTreeBbox
{
public:
    octomap::OcTreeKey min, max;

    OcTreeBbox(octomap::OcTreeKey const & min, octomap::OcTreeKey const & max) : min(min), max(max) {}

    bool contains(octomap::OcTreeKey const & p) const
    {
        for(unsigned int i = 0; i < 3; ++i)
        {
            if(!(min[i] <= p[i] && max[i] >= p[i])) return false;
        }
        return true;
    }
};

class OcTreeROI
{
public:
    std::vector<OcTreeBbox> elements;

    bool contains(octomap::OcTreeKey const & p) const
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

