#ifndef SPATIAL_REASONING_UTILS_H
#define SPATIAL_REASONING_UTILS_H

class DiscreteAngleKeymaker
{
public:
    DiscreteAngleKeymaker(double min_azimuth,
                          double max_azimuth,
                          double min_inclination,
                          double max_inclination,
                          double resolution);

    int getKey(double const & azimuth, double const & inclination) const;
    void getAngles(int const key, double & azimuth, double & inclination) const;
    int getBinCount() const;

private:
    double const min_azimuth;
    double const max_azimuth;
    double const min_inclination;
    double const max_inclination;
    double const resolution;
    int const azimuth_bin_count;
    int const inclination_bin_count;
    int const bin_count;
};

#endif // SPATIAL_REASONING_UTILS_H
