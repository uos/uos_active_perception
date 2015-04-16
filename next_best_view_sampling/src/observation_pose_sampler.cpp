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

#include "observation_pose_sampler.h"
#include "octree_regions.h"

#include <stdexcept>

struct ObservationPoseSampler::ObservationConstraints
{
    static const double EPS = 0.001;
    double rxmin, rymin, rzmin, nx, ny, nz;
    Interval b, r, a, x, y, z;

    ObservationConstraints(const CameraConstraints & cc, octomath::Vector3 n, double h)
        // init min ranges for single coordinates to 0
        : rxmin(0)
        , rymin(0)
        , rzmin(0)
        // Init normal and normal angle for degenerate case
        , nx(0)
        , ny(0)
        , nz(1)
        , b(-1, 1)
        // Init range and coords using range constraints
        , r(cc.range_min, cc.range_max)
        , x(-cc.range_max, cc.range_max)
        , y(-cc.range_max, cc.range_max)
        , z(boost::numeric::intersect(Interval(-cc.range_max, cc.range_max),
                                      Interval(cc.height_min, cc.height_max) - h))
    {
        // Init a, avoid boost::interval trig functions.
        // Extend pitch limit to allow target to be at the border of the FOV
        double a1 = -std::cos(PI/2 - (cc.pitch_min - cc.vfov / 2.0));
        double a2 = -std::cos(PI/2 - (cc.pitch_max + cc.vfov / 2.0));
        a = (a1 < a2) ? Interval(a1, a2) : Interval(a2, a1);

        if(n.norm() > EPS)
        {
            // Valid normal vector -- initialize fields and add b-constraint
            n.normalize();
            nx = n.x();
            ny = n.y();
            nz = n.z();
            b = Interval(0, 1);
        }
        converge();
    }

    /** Update intervals of all variables; Returns whether some interval changed. */
    bool update()
    {
        using namespace boost::numeric;

        double wr = width(r),
               wa = width(a),
               wb = width(b),
               wx = width(x),
               wy = width(y),
               wz = width(z);

        // enforce range/normal-angle/coords
        r = intersect(r, (nx*x + ny*y + nz*z) / b);
        b = intersect(b, (nx*x + ny*y + nz*z) / r);
        if(std::abs(nx) > EPS) x = intersect(x, (r*b - ny*y - nz*z) / nx);
        if(std::abs(ny) > EPS) y = intersect(y, (r*b - nx*x - nz*z) / ny);
        if(std::abs(nz) > EPS) z = intersect(z, (r*b - ny*y - nx*x) / nz);

        // enforce range/coords
        r = intersect(r, sqrt(square(x) + square(y) + square(z)));
        Interval ans;
        ans = sqrt(square(r) - square(y) - square(z));
        x = intersect(x, hull(-ans, ans));
        ans = sqrt(square(r) - square(x) - square(z));
        y = intersect(y, hull(-ans, ans));
        ans = sqrt(square(r) - square(x) - square(y));
        z = intersect(z, hull(-ans, ans));

        // enforce range/height/pitch
        r = intersect(r, z/a);
        a = intersect(a, z/r);
        z = intersect(z, r*a);

        // set single coord min ranges
        rxmin = std::max(x.lower(),
                         std::sqrt(std::max(0.0, std::pow(r.lower(), 2) - square(y).upper() - square(z).upper())));
        rymin = std::max(y.lower(),
                         std::sqrt(std::max(0.0, std::pow(r.lower(), 2) - square(x).upper() - square(z).upper())));
        rzmin = std::max(z.lower(),
                         std::sqrt(std::max(0.0, std::pow(r.lower(), 2) - square(x).upper() - square(y).upper())));

        return std::abs(wr - width(r)) > EPS ||
               std::abs(wa - width(a)) > EPS ||
               std::abs(wb - width(b)) > EPS ||
               std::abs(wx - width(x)) > EPS ||
               std::abs(wy - width(y)) > EPS ||
               std::abs(wz - width(z)) > EPS;
    }

    void converge()
    {
        while(update()) {}
    }

    bool isUnsolvable()
    {
        return (empty(b) || empty(r) || empty(a) || empty(x) || empty(y) || empty(z));
    }
};

ObservationPoseSampler::ObservationPoseSampler(CameraConstraints const & cam_constraints, double range_tolerance, unsigned long int seed)
:
    m_camera_constraints(cam_constraints),
    m_rng(seed),
    m_rand_u01()
{
    m_camera_constraints.range_max -= range_tolerance;
}

tf::Transform ObservationPoseSampler::genObservationSample
(
    octomath::Vector3 const & poi,
    octomath::Vector3 normal
){
    ObservationConstraints ocs(m_camera_constraints, normal, poi.z());
    if(ocs.isUnsolvable()) throw UnobservableError();

    // Prepare a randomized list of interval pointers for assignment.
    std::vector<std::pair<Interval*, double*> > xyz(3);
    // Assigning z first leads to fewer inconsistencies, so let's prefer that for now
    // switch(static_cast<int>(m_rand_u01(m_rng)*3))
    switch(2)
    {
    case 0:
        xyz[0] = std::make_pair(&ocs.x, &ocs.rxmin);
        if(m_rand_u01(m_rng) < 0.5)
        {
            xyz[1] = std::make_pair(&ocs.y, &ocs.rymin);
            xyz[2] = std::make_pair(&ocs.z, &ocs.rzmin);
        }
        else
        {
            xyz[1] = std::make_pair(&ocs.z, &ocs.rzmin);
            xyz[2] = std::make_pair(&ocs.y, &ocs.rymin);
        }
        break;
    case 1:
        xyz[0] = std::make_pair(&ocs.y, &ocs.rymin);
        if(m_rand_u01(m_rng) < 0.5)
        {
            xyz[1] = std::make_pair(&ocs.x, &ocs.rxmin);
            xyz[2] = std::make_pair(&ocs.z, &ocs.rzmin);
        }
        else
        {
            xyz[1] = std::make_pair(&ocs.z, &ocs.rzmin);
            xyz[2] = std::make_pair(&ocs.x, &ocs.rxmin);
        }
        break;
    case 2:
        xyz[0] = std::make_pair(&ocs.z, &ocs.rzmin);
        if(m_rand_u01(m_rng) < 0.5)
        {
            xyz[1] = std::make_pair(&ocs.y, &ocs.rymin);
            xyz[2] = std::make_pair(&ocs.x, &ocs.rxmin);
        }
        else
        {
            xyz[1] = std::make_pair(&ocs.x, &ocs.rxmin);
            xyz[2] = std::make_pair(&ocs.y, &ocs.rymin);
        }
        break;
    }

    for(size_t i = 0; i < 3; ++i)
    {
        Interval & iv = *(xyz[i].first);
        double & rmin = *(xyz[i].second);

        Interval ivneg(iv.lower(), std::min(iv.upper(), -rmin));
        Interval ivpos(std::max(iv.lower(), rmin), iv.upper());
        double wneg = width(ivneg);
        double wpos = width(ivpos);
        if(m_rand_u01(m_rng) < (wneg / (wneg+wpos)))
        {
            iv = Interval(ivneg.lower() + m_rand_u01(m_rng) * wneg);
        }
        else
        {
            iv = Interval(ivpos.lower() + m_rand_u01(m_rng) * wpos);
        }
        ocs.converge();
        if(ocs.isUnsolvable()) throw SamplingError();
    }

    // Extract position
    tf::Point position;
    position.setX(poi.x() + ocs.x.lower());
    position.setY(poi.y() + ocs.y.lower());
    position.setZ(poi.z() + ocs.z.lower());

    // Extract pitch and yaw
    double pitch = std::acos(ocs.a.lower()) - PI/2.0;
    double yaw = std::atan2(-ocs.y.lower(), -ocs.x.lower());

    // Limit pitch to sensor limits
    if(pitch < m_camera_constraints.pitch_min) pitch = m_camera_constraints.pitch_min;
    if(pitch > m_camera_constraints.pitch_max) pitch = m_camera_constraints.pitch_max;

    // Point the created pose towards the target voxel
    // We want to do intrinsic YPR which is equivalent to fixed axis RPY
    tf::Quaternion orientation;
    orientation.setRPY(m_camera_constraints.roll, -pitch, yaw);

    return tf::Transform(orientation, position);
}

tf::Transform ObservationPoseSampler::genPoseSample(octomath::Vector3 const & observation_position, bool lock_height)
{
    tf::Vector3 position(observation_position.x(), observation_position.y(), observation_position.z());
    if(!lock_height)
    {
        position.setZ(m_camera_constraints.height_min +
                      m_rand_u01(m_rng) * (m_camera_constraints.height_max - m_camera_constraints.height_min));
    }

    double pitch =  m_camera_constraints.pitch_min +
                    m_rand_u01(m_rng) * (m_camera_constraints.pitch_max - m_camera_constraints.pitch_min);
    double yaw = m_rand_u01(m_rng) * 2.0 * PI;
    // We want to do intrinsic YPR which is equivalent to fixed axis RPY
    tf::Quaternion orientation;
    orientation.setRPY(m_camera_constraints.roll, -pitch, yaw);

   return tf::Transform(orientation, position);
}
