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
        return (max[0] - min[0] + 1) * (max[1] - min[1] + 1) * (max[2] - min[2] + 1);
    }
};

class OcTreeBoxSet
{
public:
    std::vector<OcTreeBbox> elements;

    /**
      If the box at elements[i] contains the given key p, this function returns i+1.
      If no box contains p, 0 is returned.
      If multiple boxes in elements overlap at p, it is not defined which of the boxes is returned.
      */
    unsigned int getContainingBoxId(octomap::OcTreeKey const & p) const
    {
        for(unsigned int i = 0; i < elements.size(); ++i)
        {
            if(elements[i].contains(p))
            {
                return i+1;
            }
        }
        return 0;
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

