#include "spatial_reasoning_utils.h"

DiscreteAngleKeymaker::DiscreteAngleKeymaker(double min_azimuth,
                                             double max_azimuth,
                                             double min_inclination,
                                             double max_inclination,
                                             double resolution) :
    min_azimuth(min_azimuth),
    max_azimuth(max_azimuth),
    min_inclination(min_inclination),
    max_inclination(max_inclination),
    resolution(resolution),
    azimuth_bin_count((max_azimuth - min_azimuth) / resolution),
    inclination_bin_count((max_inclination - min_inclination) / resolution),
    bin_count(azimuth_bin_count * inclination_bin_count)
{}

DiscreteAngleKeymaker::getBinCount() const
{
    return bin_count;
}

DiscreteAngleKeymaker::getKey(const double &azimuth, const double &inclination) const
{
    int azimuth_key = (azimuth - min_azimuth) / resolution;
    int inclination_key = (inclination - min_inclination) / resolution;
    return azimuth_key * inclination_bin_count + inclination_key;
}

DiscreteAngleKeymaker::getAngles(const int key, double &azimuth, double &inclination) const
{
    inclination = resolution * (key % inclination_bin_count);
    azimuth     = resolution * (key / inclination_bin_count);
}
