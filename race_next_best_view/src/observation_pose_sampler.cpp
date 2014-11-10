#include "observation_pose_sampler.h"

#include <stdexcept>


ObservationPoseSampler::ObservationPoseSampler(CameraConstraints const & cam_constraints, double range_tolerance, unsigned long int seed)
:
    m_camera_constraints(cam_constraints),
    m_rng(seed),
    m_rand_u01()
{
    m_camera_constraints.range_max -= range_tolerance;
}

tf::Transform ObservationPoseSampler::genObservationSample(octomath::Vector3 const & poi)
{
    using boost::numeric::intersect;

    // Hardware constraints
    Interval r(m_camera_constraints.range_min, m_camera_constraints.range_max);
    Interval dH(poi.z() - m_camera_constraints.height_max, poi.z() - m_camera_constraints.height_min);
    Interval p(m_camera_constraints.pitch_min - m_camera_constraints.vfov / 2.0,
               m_camera_constraints.pitch_max + m_camera_constraints.vfov / 2.0);
    Interval sin_p = boost::numeric::sin(p);
    // Possible view angles according to hardware constraints
    sin_p = intersect(sin_p, dH / r);

    if(boost::numeric::empty(sin_p))
    {
        throw std::runtime_error("POI is unobservable!");
    }

    // Select a distance and height (randomize selection order)
    double distance, dHeight;
    if(m_rand_u01(m_rng) < 0.5)
    {
        // possible ranges that satisfy view distance, height and pitch constraints
        r = intersect(r, dH / sin_p);
        distance = r.lower() + m_rand_u01(m_rng) * (r.upper() - r.lower());
        dH = intersect(dH, distance * sin_p);
        dHeight = dH.lower() + m_rand_u01(m_rng) * (dH.upper() - dH.lower());
    }
    else
    {
        // possible height offsets that satisfy view distance, height and pitch constraints
        dH = intersect(dH, r * sin_p);
        dHeight = dH.lower() + m_rand_u01(m_rng) * (dH.upper() - dH.lower());
        r = intersect(r, dHeight / sin_p);
        distance = r.lower() + m_rand_u01(m_rng) * (r.upper() - r.lower());
    }

    // Create random position within feasible range and height
    double direction = m_rand_u01(m_rng) * 2.0 * PI;
    double x_offset = distance * std::cos(direction);
    double y_offset = distance * std::sin(direction);
    tf::Vector3 position(poi.x() + x_offset,
                         poi.y() + y_offset,
                         poi.z() - dHeight);

    // Adjust camera pitch
    double pitch = std::asin(dHeight / distance);
    if(pitch > m_camera_constraints.pitch_max)
    {
        pitch = m_camera_constraints.pitch_max;
    }
    else if(pitch < m_camera_constraints.pitch_min)
    {
        pitch = m_camera_constraints.pitch_min;
    }

    // Point the created pose towards the target voxel
    // We want to do intrinsic YPR which is equivalent to fixed axis RPY
    tf::Quaternion orientation;
    orientation.setRPY(m_camera_constraints.roll, -pitch, PI + direction);

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
