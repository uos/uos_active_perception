#ifndef ROS_SERIALIZATION_HELPER_H
#define ROS_SERIALIZATION_HELPER_H

#include <ros/serialization.h>

#include <string>
#include <fstream>

namespace ros_serialization_helper
{
    template<class T>
    bool readSerialized(const std::string & f, T & data)
    {
        std::ifstream iss(f.c_str(), std::ios::binary);
        if(!iss.good()) return false;
        iss.seekg (0, std::ios::end);
        std::streampos end = iss.tellg();
        iss.seekg (0, std::ios::beg);
        std::streampos begin = iss.tellg();
        size_t file_size = end-begin;
        boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
        iss.read((char*) ibuffer.get(), file_size);
        ros::serialization::IStream istream(ibuffer.get(), file_size);
        ros::serialization::deserialize(istream, data);
        return true;
    }

    template<class T>
    void writeSerialized(const std::string & f, const T & data)
    {
        std::ofstream oss(f.c_str(), std::ios::binary);
        size_t serial_size = ros::serialization::serializationLength(data);
        boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
        ros::serialization::OStream ostream(obuffer.get(), serial_size);
        ros::serialization::serialize(ostream, data);
        oss.write((char*) obuffer.get(), serial_size);
    }
}

#endif // ROS_SERIALIZATION_HELPER_H
