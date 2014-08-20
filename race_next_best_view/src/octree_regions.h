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

    unsigned int cellCount() const
    {
        return max[0] - min[0] + 1 * max[1] - min[1] + 1 * max[2] - min[2] + 1;
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

    unsigned int cellCount() const
    {
        unsigned int size = 0;
        for(std::vector<OcTreeBbox>::const_iterator it = elements.begin(); it != elements.end(); ++it)
        {
            size += it->cellCount();
        }
        return size;
    }
};

#endif // OCTREE_REGIONS_H

