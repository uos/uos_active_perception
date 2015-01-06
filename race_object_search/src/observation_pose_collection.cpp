#include "observation_pose_collection.h"

#include <fstream>

void ObservationPoseCollection::addPoses
(
    std::vector<geometry_msgs::Pose> const & new_poses,
    std::vector<geometry_msgs::Point> const & new_target_points,
    std::vector<uos_active_perception_msgs::ConditionalVisibilityMap> const & new_cvms,
    std::vector<uos_active_perception_msgs::ObjectSet> const & new_object_sets
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
        tf::Point target_point;
        tf::pointMsgToTF(new_target_points[i], target_point);
        op.view_distance = op.pose.getOrigin().distance(target_point);
        op.object_set_ids = new_cvms[i].object_set_ids;
        op.cell_id_sets.reserve(new_cvms[i].cell_id_sets.size());
        for(size_t j = 0; j < new_cvms[i].cell_id_sets.size(); ++j) {
            op.cell_id_sets.push_back(boost::unordered_set<uint64_t>(new_cvms[i].cell_id_sets[j].cell_ids.begin(),
                                                                     new_cvms[i].cell_id_sets[j].cell_ids.end()));
        }
        m_observation_poses.push_back(op);
    }
}

std::vector<ObservationPose> const & ObservationPoseCollection::getPoses() const
{
    return m_observation_poses;
}

double ObservationPoseCollection::getInitialTravelTime(size_t target_idx) const
{
    return m_initial_travel_time_lut[target_idx];
}

double ObservationPoseCollection::getTravelTime(size_t start_idx, size_t target_idx) const
{
    if(start_idx == (size_t) -1) {
        return getInitialTravelTime(target_idx);
    } else if(target_idx == (size_t) -1) {
        return getInitialTravelTime(start_idx);
    } else {
        return m_travel_time_lut[getTtLutIdx(start_idx, target_idx)];
    }
}

void ObservationPoseCollection::dumpInitialTravelTimeMap() const
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

size_t ObservationPoseCollection::pruneUnreachablePoses()
{
    size_t n_pruned = 0;
    std::vector<ObservationPose> pruned_observation_poses;
    std::vector<double> pruned_initial_travel_time_lut;
    pruned_observation_poses.reserve(m_observation_poses.size());
    pruned_initial_travel_time_lut.reserve(m_initial_travel_time_lut.size());
    for(size_t i = 0; i < m_observation_poses.size(); ++i) {
        if(m_initial_travel_time_lut[i] < std::numeric_limits<double>::infinity()) {
            pruned_observation_poses.push_back(m_observation_poses[i]);
            pruned_initial_travel_time_lut.push_back(m_initial_travel_time_lut[i]);
        } else {
            ++n_pruned;
        }
    }
    m_observation_poses = pruned_observation_poses;
    m_initial_travel_time_lut = pruned_initial_travel_time_lut;
    return n_pruned;
}
