#ifndef OBSERVATION_POSE_SAMPLER_H
#define OBSERVATION_POSE_SAMPLER_H

#include "camera_constraints.h"

#include <tf/tf.h>
#include <octomap/octomap.h>
#include <boost/random.hpp>
#include <boost/numeric/interval.hpp>

#include <ctime>

class ObservationPoseSampler
{
public:
    /**
      ASSUMPTION: Camera constraints are defined in the world frame.
      */
    ObservationPoseSampler(CameraConstraints const & cam_constraints, double range_tolerance, unsigned long seed = std::time(0));

    tf::Transform genObservationSample(octomath::Vector3 const & poi);

    tf::Transform genPoseSample(octomath::Vector3 const & observation_position, bool lock_height);


private:
    static const double PI = 3.1415926535897932384626433832795028841971693993751058209;

    CameraConstraints m_camera_constraints;
    boost::mt19937 m_rng;
    boost::uniform_01<> m_rand_u01;

    typedef boost::numeric::interval<
                double,
                boost::numeric::interval_lib::policies<
                    boost::numeric::interval_lib::save_state<
                        boost::numeric::interval_lib::rounded_transc_std<double> >,
                    boost::numeric::interval_lib::checking_base<double> > >
            Interval;
};

#endif // OBSERVATION_POSE_SAMPLER_H
