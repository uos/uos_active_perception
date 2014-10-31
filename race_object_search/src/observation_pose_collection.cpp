#include "observation_pose_collection.h"

#include <fstream>

void ObservationPoseCollection::addPoses
(
    std::vector<geometry_msgs::Pose> const & new_poses,
    std::vector<race_next_best_view::ConditionalVisibilityMap> const & new_cvms,
    std::vector<race_next_best_view::ObjectSet> const & new_object_sets
){
    // Integrate new object sets
    // TODO: Keep object sets and map new object sets to old ones (not necessary while there are no objects)
    m_object_sets.clear();
    m_object_sets.reserve(new_object_sets.size());
    for(size_t i = 0; i < new_object_sets.size(); ++i) {
        m_object_sets.push_back(new_object_sets[i].objects);
    }

    // Integrate new poses
    for(size_t i = 0; i < new_poses.size(); ++i) {
        ObservationPose op;
        tf::poseMsgToTF(new_poses[i], op.pose);
        op.object_set_ids = new_cvms[i].object_set_ids;
        op.cell_id_sets.reserve(new_cvms[i].cell_id_sets.size());
        for(size_t j = 0; j < new_cvms[i].cell_id_sets.size(); ++j) {
            op.cell_id_sets.push_back(boost::unordered_set<uint64_t>(new_cvms[i].cell_id_sets[j].cell_ids.begin(),
                                                                     new_cvms[i].cell_id_sets[j].cell_ids.end()));
        }
        m_observation_poses.push_back(op);
    }
}

std::vector<ObservationPose> const & ObservationPoseCollection::getPoses()
{
    return m_observation_poses;
}

double ObservationPoseCollection::getInitialTravelTime(size_t target_idx)
{
    return m_initial_travel_time_lut[target_idx];
}

double ObservationPoseCollection::getTravelTime(size_t start_idx, size_t target_idx)
{
    return m_travel_time_lut[getTtLutIdx(start_idx, target_idx)];
}

void ObservationPoseCollection::dumpInitialTravelTimeMap()
{
    std::ofstream f;
    f.open ("initial_travel_times.tab");
    f << "x\ty\ttime\n";
    f << "c\tc\tc\n";
    f << "\t\tc\n";
    for(size_t i = 0; i < m_observation_poses.size(); ++i) {
        f << m_observation_poses[i].pose.getOrigin().getX() << "\t" << m_observation_poses[i].pose.getOrigin().getY() << "\t" << m_initial_travel_time_lut[i] << "\n";
    }
    f.close();
}
