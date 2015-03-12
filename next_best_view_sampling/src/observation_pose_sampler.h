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

    tf::Transform genObservationSample(octomath::Vector3 const & poi, octomath::Vector3 normal);

    tf::Transform genPoseSample(octomath::Vector3 const & observation_position, bool lock_height);


private:
    typedef boost::numeric::interval<
                double,
                boost::numeric::interval_lib::policies<
                    boost::numeric::interval_lib::save_state<
                        boost::numeric::interval_lib::rounded_transc_std<double> >,
                    boost::numeric::interval_lib::checking_catch_nan<double> > >
            Interval;

    struct ObservationConstraints;

    static const double PI = 3.1415926535897932384626433832795028841971693993751058209;

    CameraConstraints m_camera_constraints;
    boost::mt19937 m_rng;
    boost::uniform_01<> m_rand_u01;
};

#endif // OBSERVATION_POSE_SAMPLER_H
