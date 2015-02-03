#ifndef OBSERVATION_POSE_COLLECTION_H
#define OBSERVATION_POSE_COLLECTION_H

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <boost/unordered_set.hpp>
#include <uos_active_perception_msgs/ConditionalVisibilityMap.h>
#include <uos_active_perception_msgs/ObjectSet.h>

#include <vector>
#include <cassert>
#include <stdint.h>

typedef boost::unordered_set<uint64_t> detection_t;

struct ObservationPose
{
    tf::Pose pose;
    std::vector<detection_t> cell_id_sets;
    std::vector<uint32_t> object_set_ids;
    double view_distance;
};

class ObservationPoseCollection
{
public:
    void addPoses(std::vector<geometry_msgs::Pose> const & new_poses,
                  std::vector<geometry_msgs::Point> const & new_target_points,
                  std::vector<uos_active_perception_msgs::ConditionalVisibilityMap> const & new_cvms,
                  std::vector<uos_active_perception_msgs::ObjectSet> const & new_object_sets);

    std::vector<ObservationPose> const & getPoses() const;

    double getInitialTravelTime(size_t target_idx) const;

    double getTravelTime(size_t start_idx, size_t target_idx) const;

    void dumpInitialTravelTimeMap(const std::string & fname) const;

    size_t pruneUnreachablePoses();

    template <class Agent>
    void prepareInitialTravelTimeLut
    (
            Agent const & agent,
            tf::Pose const & initial_base_pose,
            tf::Pose const & initial_cam_pose,
            std::string const & world_frame_id
    ){
        std::vector<tf::Pose> cam_poses;
        cam_poses.reserve(m_observation_poses.size() + 1);
        for(size_t i = 0; i < m_observation_poses.size(); ++i) {
            cam_poses.push_back(m_observation_poses[i].pose);
        }
        cam_poses.push_back(initial_cam_pose);

        std::vector<tf::Pose> base_poses;
        base_poses.reserve(m_observation_poses.size() + 1);
        for(size_t i = 0; i < m_observation_poses.size(); ++i) {
            base_poses.push_back(agent.robotPoseForCamPose(m_observation_poses[i].pose));
        }
        base_poses.push_back(initial_base_pose);


        std::vector<size_t> start_pose_idxs(m_observation_poses.size(), m_observation_poses.size());
        std::vector<size_t> target_pose_idxs(m_observation_poses.size());
        for(size_t i = 0; i < m_observation_poses.size(); ++i) {
            target_pose_idxs[i] = i;
        }

        m_initial_travel_time_lut = agent.estimateMoveTimes(cam_poses,
                                                            base_poses,
                                                            start_pose_idxs,
                                                            target_pose_idxs,
                                                            cam_poses.size());
    }

    template <class Agent>
    void prepareTravelTimeLut
    (
            Agent const & agent,
            std::string const & world_frame_id
    ){
        size_t n = m_observation_poses.size();
        if(!n) return;
        size_t lut_size = getTtLutIdx(n, 0);

        std::vector<tf::Pose> cam_poses;
        cam_poses.reserve(n);
        for(size_t i = 0; i < n; ++i) {
            cam_poses.push_back(m_observation_poses[i].pose);
        }

        std::vector<tf::Pose> base_poses;
        base_poses.reserve(n);
        for(size_t i = 0; i < n; ++i) {
            base_poses.push_back(agent.robotPoseForCamPose(m_observation_poses[i].pose));
        }

        std::vector<size_t> start_pose_idxs(lut_size);
        std::vector<size_t> target_pose_idxs(lut_size);

        for(size_t i = 0; i < n; ++i) {
            for(size_t j = 0; j < i; ++j) {
                size_t lut_idx = getTtLutIdx(i,j);
                start_pose_idxs[lut_idx] = i;
                target_pose_idxs[lut_idx] = j;
            }
        }

        m_travel_time_lut = agent.estimateMoveTimes(cam_poses,
                                                    base_poses,
                                                    start_pose_idxs,
                                                    target_pose_idxs,
                                                    50);
    }

    bool sanityCheck()
    {
        size_t ii, jj, kk;
        double failtime = 0.0;
        for(size_t i = 0; i < m_observation_poses.size(); ++i)
        {
            for(size_t j = 0; j < i; ++j)
            {
                for(size_t k = 0; k < m_observation_poses.size(); ++k)
                {
                    if(k == i || k == j) continue;
                    double tt = getTravelTime(i, j);
                    if((tt < std::numeric_limits<double>::infinity()) &&
                       (tt > (getTravelTime(i, k) + getTravelTime(k, j))))
                    {
                        double fail = (tt - (getTravelTime(i, k) + getTravelTime(k, j)));
                        if(fail > failtime)
                        {
                            failtime = fail;
                            ii = i;
                            jj = j;
                            kk = k;
                        }
                    }
                }
            }
        }
        if(failtime > 0) {
            geometry_msgs::Pose a,b,c;
            tf::poseTFToMsg(m_observation_poses[ii].pose, a);
            tf::poseTFToMsg(m_observation_poses[jj].pose, b);
            tf::poseTFToMsg(m_observation_poses[kk].pose, c);
            ROS_ERROR_STREAM("Sanity check failed, indirect route is " << failtime << " faster!" <<
                             "\n--- Long path\nStart:\n" << a << "\nGoal:\n" << b <<
                             "\n--- Faster[1]:\nStart:\n" << a << "\nGoal:\n" << c <<
                             "\n--- Faster[2]:\nStart:\n" << c << "\nGoal:\n" << b);
            return false;
        }

        return true;
    }

    detection_t observableUnion()
    {
        detection_t all;
        for(size_t i = 0; i < m_observation_poses.size(); ++i)
        {
            for(size_t j = 0; j < m_observation_poses[i].cell_id_sets.size(); ++j)
            {
                detection_t & ins = m_observation_poses[i].cell_id_sets[j];
                all.insert(ins.begin(), ins.end());
            }
        }
        return all;
    }

    static uint64_t cellIdMsgToInt(const uos_active_perception_msgs::CellId & msg)
    {
        uint64_t id = 0;
        id |= msg.x;
        id <<= 16;
        id |= msg.y;
        id <<= 16;
        id |= msg.z;
        return id;
    }

    static uos_active_perception_msgs::CellId cellIdIntToMsg(uint64_t id)
    {
        uos_active_perception_msgs::CellId msg;
        msg.z = id;
        id >>= 16;
        msg.y = id;
        id >>= 16;
        msg.x = id;
        return msg;
    }


private:
    std::vector<std::vector<unsigned int> > m_object_sets;
    std::vector<ObservationPose> m_observation_poses;
    std::vector<double> m_initial_travel_time_lut;
    std::vector<double> m_travel_time_lut;

    static size_t getTtLutIdx(size_t row, size_t col)
    {
        assert(row != col);
        if(row > col)
            return (((row - 1) * row) / 2) + col;
        else
            return (((col - 1) * col) / 2) + row;
    }
};

#endif // OBSERVATION_POSE_COLLECTION_H
