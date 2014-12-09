#ifndef OBSERVATION_POSE_COLLECTION_H
#define OBSERVATION_POSE_COLLECTION_H

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <boost/unordered_set.hpp>
#include <race_next_best_view/ConditionalVisibilityMap.h>
#include <race_next_best_view/ObjectSet.h>

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
                  std::vector<race_next_best_view::ConditionalVisibilityMap> const & new_cvms,
                  std::vector<race_next_best_view::ObjectSet> const & new_object_sets);

    std::vector<ObservationPose> const & getPoses() const;

    double getInitialTravelTime(size_t target_idx) const;

    double getTravelTime(size_t start_idx, size_t target_idx) const;

    void dumpInitialTravelTimeMap() const;

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
            base_poses.push_back(agent.robot_pose_for_cam_pose(m_observation_poses[i].pose));
        }
        base_poses.push_back(initial_base_pose);


        std::vector<size_t> start_pose_idxs(m_observation_poses.size(), m_observation_poses.size());
        std::vector<size_t> target_pose_idxs(m_observation_poses.size());
        for(size_t i = 0; i < m_observation_poses.size(); ++i) {
            target_pose_idxs[i] = i;
        }

        m_initial_travel_time_lut = agent.estimate_move_times(cam_poses,
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
            base_poses.push_back(agent.robot_pose_for_cam_pose(m_observation_poses[i].pose));
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

        m_travel_time_lut = agent.estimate_move_times(cam_poses,
                                                      base_poses,
                                                      start_pose_idxs,
                                                      target_pose_idxs,
                                                      50);
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
