#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <uos_active_perception_msgs/CellId.h>
#include <uos_active_perception_msgs/CellIds.h>
#include <boost/unordered_set.hpp>
#include <boost/random.hpp>

#include <vector>

namespace geometry
{
    typedef boost::unordered_set<uint64_t> detection_t;

    inline uint64_t cellIdMsgToInt(const uos_active_perception_msgs::CellId & msg)
    {
        uint64_t id = 0;
        id |= msg.x;
        id <<= 16;
        id |= msg.y;
        id <<= 16;
        id |= msg.z;
        return id;
    }

    inline uos_active_perception_msgs::CellId cellIdIntToMsg(uint64_t id)
    {
        uos_active_perception_msgs::CellId msg;
        msg.z = id;
        id >>= 16;
        msg.y = id;
        id >>= 16;
        msg.x = id;
        return msg;
    }

    inline detection_t cellIdsMsgToDetection(const uos_active_perception_msgs::CellIds & msg)
    {
        detection_t out;
        for(size_t k = 0; k < msg.cell_ids.size(); ++k)
        {
            out.insert(cellIdMsgToInt(msg.cell_ids[k]));
        }
        return out;
    }

    struct MapRegion
    {
        uos_active_perception_msgs::CellId min, max;
        bool contains(uos_active_perception_msgs::CellId cell) const
        {
            return min.x <= cell.x && cell.x <= max.x &&
                   min.y <= cell.y && cell.y <= max.y &&
                   min.z <= cell.z && cell.z <= max.z;
        }
        size_t cellCount()
        {
            return (max.x - min.x + 1) * (max.y - min.y + 1) * (max.z - min.z + 1);
        }
    };

    struct MapRegionCollection
    {
        std::vector<MapRegion> regions;
        size_t findRegion(uos_active_perception_msgs::CellId cell) const
        {
            for(size_t i = 0; i < regions.size(); ++i)
            {
                if(regions[i].contains(cell)) return i;
            }
            return -1;
        }
    };

    inline bool testCellIdIntMsgConversion(size_t n = 9000)
    {
        boost::mt19937 rng(std::time(0));
        boost::uniform_int<uint16_t> rand(0, std::numeric_limits<uint16_t>::max());
        for(size_t i = 0; i < n; ++i)
        {
            uos_active_perception_msgs::CellId msg;
            msg.x = rand(rng);
            msg.y = rand(rng);
            msg.z = rand(rng);
            uint64_t id = cellIdMsgToInt(msg);
            uos_active_perception_msgs::CellId msg2 = cellIdIntToMsg(id);
            if(msg2.x != msg.x || msg2.y != msg.y || msg2.z != msg.z)
            {
                return false;
            }
        }
        return true;
    }
}

#endif // GEOMETRY_H
