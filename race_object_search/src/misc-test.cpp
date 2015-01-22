#include "observation_pose_collection.h"

#include <boost/random.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <stdint.h>
#include <limits>
#include <iostream>
#include <ctime>

int main()
{
    std::cout << "sizeof long is: " << sizeof(long) << std::endl;

    size_t n = 9000;
    boost::mt19937 rng(std::time(0));
    boost::uniform_int<uint16_t> rand(0, std::numeric_limits<uint16_t>::max());
    std::cout << "Testing " << n << " random msgs..." << std::endl;
    for(size_t i = 0; i < n; ++i)
    {
        uos_active_perception_msgs::CellId msg;
        msg.x = rand(rng);
        msg.y = rand(rng);
        msg.z = rand(rng);
        uint64_t id = ObservationPoseCollection::cellIdMsgToInt(msg);
        uos_active_perception_msgs::CellId msg2 = ObservationPoseCollection::cellIdIntToMsg(id);
        if(msg2.x != msg.x || msg2.y != msg.y || msg2.z != msg.z)
        {
            std::cerr << "Reconstructed msg not equal!" << std::endl;
            break;
        }
    }

    std::cout << "Test finished" << std::endl;

    return 0;
}
