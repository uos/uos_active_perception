#ifndef OCTREE_RAY_ITERATOR_H
#define OCTREE_RAY_ITERATOR_H

#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeBase.h>
#include <octomap/math/Vector3.h>

#include <stdexcept>

/**
  An iterator to perform step-by-step ray casts in octrees.
  */
class RayIterator
// Implementation stolen from OcTreeBaseImpl.hxx
{
public:
    template <class NODE, class I>
    RayIterator
    (
        octomap::OcTreeBaseImpl<NODE, I> const & octree,
        octomath::Vector3 const & origin_param,
        octomath::Vector3 const & direction_param)
    :
        origin(origin_param),
        direction(direction_param.normalized())
    {
        // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
        // basically: DDA in 3D
        using namespace octomap;
        using namespace octomath;

        if(!octree.coordToKeyChecked(origin, current_key))
        {
            throw std::out_of_range("ray origin coordinates out of bounds");
        }

        for(unsigned int i = 0; i < 3; ++i)
        {
            // compute step direction
            if (direction(i) > 0.0) step[i] =  1;
            else if (direction(i) < 0.0)   step[i] = -1;
            else step[i] = 0;

            // compute tMax, tDelta
            if (step[i] != 0)
            {
                // corner point of voxel (in direction of ray)
                double voxelBorder = octree.keyToCoord(current_key[i]);
                voxelBorder += (float) (step[i] * octree.getResolution() * 0.5);

                tMax[i] = (voxelBorder - origin(i)) / direction(i);
                tDelta[i] = octree.getResolution() / fabs(direction(i));
            }
            else
            {
                tMax[i] = std::numeric_limits<double>::max();
                tDelta[i] = std::numeric_limits<double>::max();
            }
        }
    }

    /**
      Advances the ray by one voxel.
      */
    void next()
    {
        unsigned int dim;
        // find minimum tMax:
        if (tMax[0] < tMax[1]){
            if (tMax[0] < tMax[2]) dim = 0;
            else                   dim = 2;
        }
        else {
            if (tMax[1] < tMax[2]) dim = 1;
            else                   dim = 2;
        }

        // TODO: Check for coordinate overflow

        // advance in direction "dim"
        current_key[dim] += step[dim];
        tMax[dim] += tDelta[dim];
    }

    /**
      Returns the key of the current voxel.
      */
    octomap::OcTreeKey const & getKey()
    {
        return current_key;
    }

    /**
      Returns the length of the ray when traveled until the border of the current voxel.
      */
    double distanceFromOrigin()
    {
        return std::min(std::min(tMax[0], tMax[1]), tMax[2]);
    }

    /**
      Returns the point where the ray enters the current voxel.
      */
    octomath::Vector3 currentPosition()
    {
        return origin + direction * distanceFromOrigin();
    }

private:
    octomath::Vector3 const origin;
    octomath::Vector3 const direction;
    int    step[3];
    double tMax[3];
    double tDelta[3];
    octomap::OcTreeKey current_key;
};

#endif // OCTREE_RAY_ITERATOR_H

