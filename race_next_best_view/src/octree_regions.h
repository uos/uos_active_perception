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

