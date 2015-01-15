#ifndef SEARCH_PLAN_H
#define SEARCH_PLAN_H

#include "observation_pose_collection.h"

#include <visualization_msgs/Marker.h>
#include <ros/publisher.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <map>

template <class CELL_GAIN_LOOKUP>
class SearchPlan
{
public:
    size_t last_idx;
    std::vector<double> time;
    std::vector<double> etime;
    std::vector<double> pdone;
    std::vector<size_t> cam_pose_idx;
    std::vector<detection_t> detected_cells;

    SearchPlan(CELL_GAIN_LOOKUP const & cgl, ObservationPoseCollection const & opc)
        : last_idx(0)
        , time(std::vector<double>(1, 0.0))
        , etime(std::vector<double>(1, 0.0))
        , pdone(std::vector<double>(1, 0.0))
        , cam_pose_idx(std::vector<size_t>(1, -1))
        , detected_cells(std::vector<detection_t>(1))
        , cgl(cgl)
        , opc(opc)
    {}

    void clear()
    {
        last_idx = 0;
        time = std::vector<double>(1, 0.0);
        etime = std::vector<double>(1, 0.0);
        pdone = std::vector<double>(1, 0.0);
        cam_pose_idx = std::vector<size_t>(1, -1);
        detected_cells = std::vector<detection_t>(1);
    }

    void pop()
    {
        --last_idx;
        time.pop_back();
        etime.pop_back();
        pdone.pop_back();
        cam_pose_idx.pop_back();
        detected_cells.pop_back();
    }

    bool push(size_t const next_cam_pose_idx)
    {
        detection_t const & new_detection = opc.getPoses()[next_cam_pose_idx].cell_id_sets[0];
        double gain = calcGain(detected_cells[last_idx], new_detection);
        if(gain < std::numeric_limits<double>::epsilon()) {
            return false;
        }
        double duration = opc.getTravelTime(cam_pose_idx[last_idx], next_cam_pose_idx);
        double eduration = (1.0 - pdone[last_idx]) * duration;
        cam_pose_idx.push_back(next_cam_pose_idx);
        time.push_back(time[last_idx] + duration);
        etime.push_back(etime[last_idx] + eduration);
        pdone.push_back(pdone[last_idx] + gain);
        detected_cells.push_back(detection_union(detected_cells[last_idx], new_detection));
        ++last_idx;
        return true;
    }

    void pushSequence(std::vector<size_t> const & cam_pose_sequence, size_t start, size_t end)
    {
        for(size_t i = start; i < end; ++i) {
            push(cam_pose_sequence[i]);
        }
    }

    // Replaced by better code in search_planner
    bool optimalOrder()
    {
        double old_etime = etime[last_idx];
        std::vector<size_t> original_sequence(cam_pose_idx.begin()+1, cam_pose_idx.end());
        std::vector<size_t> optimal_order_seq;
        double limit = std::numeric_limits<double>::infinity();//etime[last_idx];
        clear();
        optimalSequenceSearch(original_sequence, false, original_sequence.size() + 1, optimal_order_seq, limit);
        pushSequence(optimal_order_seq, 1, optimal_order_seq.size());
        return etime[last_idx] < old_etime;
    }

    // TODO: Replace with better code in search_planner
    bool optimizeSteps()
    {
        double old_etime = etime[last_idx];
        std::vector<size_t> original_sequence(cam_pose_idx.begin()+1, cam_pose_idx.end());
        clear();
        while(pushOptimalIntermediateStep(original_sequence)) {};
        return etime[last_idx] < old_etime;
    }

    // TODO: Replace with better code in search_planner
    void optimizeLocally()
    {
        bool change;
        do {
            change = optimizeSteps();
            std::cout << "Step-optimized plan has an etime of " << etime[last_idx] << std::endl;
            change = optimalOrder();
            std::cout << "Reordered plan has an etime of " << etime[last_idx] << std::endl;
        } while(change);
    }

    // Replaced by better code in search_planner
    void greedy(double const horizon)
    {
        double tmax = time[last_idx] + horizon;
        while(time[last_idx] < tmax) {
            double max_utility = 0.0;
            size_t max_utility_idx = 0;
            for(size_t i = 0; i < opc.getPoses().size(); ++i) {
                if(i == cam_pose_idx[last_idx]) continue;
                double duration = opc.getTravelTime(cam_pose_idx[last_idx], i);
                detection_t const & new_detection = opc.getPoses()[i].cell_id_sets[0];
                double gain = calcGain(detected_cells[last_idx], new_detection);
                double utility = gain / duration;
                if(utility > max_utility) {
                    max_utility = utility;
                    max_utility_idx = i;
                }
            }
            if(max_utility < std::numeric_limits<double>::epsilon()) {
                // Cannot find more useful observation poses
                break;
            } else {
                push(max_utility_idx);
            }
        }
    }

    size_t sendMarker(std::string frame_id, ros::Publisher & pub, const std::string & ns)
    {
        // Publish current markers
        for(size_t i = 1; i <= last_idx; ++i) {
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.2;
            marker.color.g = 1.0;
            marker.color.r = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;
            tf::poseTFToMsg(opc.getPoses()[cam_pose_idx[i]].pose, marker.pose);
            marker.header.frame_id = frame_id;
            marker.id = i;
            marker.ns = ns;
            pub.publish(marker);
        }
        // Connecting lines
        for(size_t i = 1; i+1 <= last_idx; ++i) {
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.02;
            marker.scale.z = std::numeric_limits<float>::epsilon();
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            geometry_msgs::Point p;
            tf::pointTFToMsg(opc.getPoses()[cam_pose_idx[i]].pose.getOrigin(), p);
            marker.points.push_back(p);
            tf::pointTFToMsg(opc.getPoses()[cam_pose_idx[i+1]].pose.getOrigin(), p);
            marker.points.push_back(p);
            marker.header.frame_id = frame_id;
            marker.id = last_idx + 1 + i;
            marker.ns = ns;
            pub.publish(marker);
        }
        return 2 * last_idx;
    }

    void deleteMarker(const std::string & frame_id, ros::Publisher & pub, const std::string & ns, const size_t n)
    {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.ns = ns;
        marker.header.frame_id = frame_id;
        for(size_t i = 1; i <= n; ++i) {
            marker.id = i;
            pub.publish(marker);
        }
    }


    void writeTimeplot(std::string const & filename)
    {
        std::ofstream f;
        f.open(filename.c_str());
        f << "pDone\ttime\tetime\n";
        for(size_t i = 0; i <= last_idx; ++i) {
            f << pdone[i] << "\t" << time[i] << "\t" << etime[i] << "\n";
        }
        f.close();
    }

private:
    CELL_GAIN_LOOKUP const & cgl;
    ObservationPoseCollection const & opc;

    double calcGain(detection_t const & a, detection_t const & b)
    {
        double gain = 0.0;
        for(detection_t::iterator it = b.begin(); it != b.end(); ++it) {
            if(!a.count(*it)) {
                gain += cgl(*it);
            }
        }
        return gain;
    }

    static detection_t detection_union(detection_t const & a, detection_t const & b)
    {
        if(a.size() >= b.size()) {
            detection_t du;
            du.rehash((a.size() + b.size()) / du.max_load_factor());
            du.insert(a.begin(), a.end());
            du.insert(b.begin(), b.end());
            return du;
        } else {
            return detection_union(b, a);
        }
    }

    bool pushOptimalIntermediateStep(std::vector<size_t> original_sequence)
    {
        size_t current_seq_len = cam_pose_idx.size();
        double best_etime = std::numeric_limits<double>::infinity();
        size_t best_idx = 0;
        for(size_t i = 0; i < opc.getPoses().size(); ++i) {
            if(push(i)) {
                pushSequence(original_sequence, 1, original_sequence.size());
                if(etime[last_idx] < best_etime) {
                    best_etime = etime[last_idx];
                    best_idx = i;
                }
                while(cam_pose_idx.size() > current_seq_len) {
                    pop();
                }
            }
        }
        if(best_etime != std::numeric_limits<double>::infinity()) {
            return push(best_idx);
        } else {
            return false;
        }
    }

    void optimalSequenceSearch(std::vector<size_t> const & opc_subset,
                               bool const optimize_locally,
                               int const max_depth,
                               std::vector<size_t> & sequence,
                               double & limit)
    {
        // Create a sorted agenda of child states to expand
        typedef std::multimap<double, std::pair<size_t, double> > cost_pose_etime_map;
        cost_pose_etime_map m;
        for(size_t i = 0; i < opc_subset.size(); ++i) {
            if(push(opc_subset[i])) {
                double gain = pdone[last_idx] - pdone[last_idx-1];
                double duration = time[last_idx] - time[last_idx-1];
                double greedy_cost = duration / gain;
                double cutoff_val = etime[last_idx];// / pdone[last_idx];
                //greedy_cost = cutoff_val;
                m.insert(std::make_pair(greedy_cost, std::make_pair(opc_subset[i], cutoff_val)));
                pop();
            }
        }

        // Test for goal state
        if(m.empty()) {
            // This state has no children and is therefore a goal state.
            // Since we got here, this is the best goal state known yet.
            if(optimize_locally) {
                std::vector<size_t> original_sequence(cam_pose_idx.begin(), cam_pose_idx.end());
                optimizeLocally();
                sequence = cam_pose_idx;
                limit = etime[last_idx];
                clear();
                pushSequence(original_sequence, 1, original_sequence.size());
            } else {
                sequence = cam_pose_idx;
                limit = etime[last_idx];
            }
            std::cout << "FOUND NEW OPTIMUM with etime " << limit << std::endl;
        }

        // Work through agenda
        double greedy_neighborhood = 1.1 * m.begin()->first;
        for(cost_pose_etime_map::iterator it = m.begin(); it != m.end(); ++it) {
            if(it->second.second > limit) {
                // etime > limit => This branch is hopeless and can be skipped
                continue;
            }
            if(it->first > greedy_neighborhood) {
                break;
            }
            push(it->second.first);
            optimalSequenceSearch(opc_subset, optimize_locally, max_depth - 1, sequence, limit);
            pop();
            if(max_depth <= 0) {
                // Beyond max_depth, we are only interested in greedy solutions and therefore cut all alternatives.
                break;
            }
            //std::cout << "BRANCH COMPLETE " << cam_pose_idx.size() << std::endl;
        }
    }
};

#endif // SEARCH_PLAN_H
