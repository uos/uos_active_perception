#ifndef SEARCH_PLAN2_H
#define SEARCH_PLAN2_H

#include "observation_pose_collection.h"

#include <visualization_msgs/Marker.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <map>


template <class CELL_GAIN_LOOKUP>
class SearchPlanner
{
public:

    SearchPlanner(CELL_GAIN_LOOKUP const & cgl, ObservationPoseCollection const & opc)
        : cgl(cgl)
        , opc(opc)
    {}

    bool makePlan(size_t const arg_depth_limit,
                  std::vector<size_t> & result_sequence,
                  double & result_etime)
    {
        opc_subset.resize(opc.getPoses().size());
        for(size_t i = 0; i < opc_subset.size(); ++i) {
            opc_subset[i] = i;
        }
        depth_limit = arg_depth_limit;
        if(memory.empty()) {
            memory.resize(1000);
        }
        memory[0].detec_opts.init(cgl, opc);
        sequence.clear();
        sequence.push_back(-1);
        best_sequence.clear();
        best_etime = std::numeric_limits<double>::infinity();

        std::cout << "Starting recursive search..." << std::endl;
        makePlanRecursive(0, 0.0, 0.0);

        result_sequence = best_sequence;
        result_etime = best_etime;
        return(!best_sequence.empty());
    }

private:
    class DetectionOptions
    {
    public:
        std::vector<detection_t> detectables;
        std::vector<double> gain;

        void init(CELL_GAIN_LOOKUP const & cgl, ObservationPoseCollection const & opc)
        {
            detectables.clear();
            detectables.reserve(opc.getPoses().size());
            gain.clear();
            gain.reserve(opc.getPoses().size());
            for(size_t i = 0; i < opc.getPoses().size(); ++i) {
                detection_t const & d = opc.getPoses()[i].cell_id_sets[0];
                detectables.push_back(d);
                double g = 0.0;
                for(detection_t::iterator it = d.begin(); it != d.end(); ++it) {
                    g += cgl(*it);
                }
                gain.push_back(g);
            }
        }

        DetectionOptions & assignEqualSize(const DetectionOptions & other)
        {
            assert(detectables.size() == other.detectables.size());
            for(size_t i = 0; i < detectables.size(); ++i) {
                detectables[i].clear();
                detectables[i].insert(other.detectables[i].begin(), other.detectables[i].end());
                gain[i] = other.gain[i];
            }
            return *this;
        }

        void removeCells(detection_t const & d, CELL_GAIN_LOOKUP const & cgl)
        {
            for(size_t i = 0; i < detectables.size(); ++i) {
                for(detection_t::iterator it = d.begin(); it != d.end(); ++it) {
                    if(detectables[i].erase(*it)) {
                        gain[i] -= cgl(*it);
                    }
                }
            }
        }
    };

    typedef std::multimap<double, std::pair<size_t, double> > expansion_map_t; // cost -> (pose, etime)

    struct SearchStateMemory
    {
        DetectionOptions detec_opts;
        expansion_map_t expansion_map;
    };

    CELL_GAIN_LOOKUP const & cgl;
    ObservationPoseCollection const & opc;
    std::vector<size_t> opc_subset;
    size_t depth_limit;
    std::vector<SearchStateMemory> memory;
    std::vector<size_t> sequence;
    std::vector<size_t> best_sequence;
    double best_etime;

    void makePlanRecursive(size_t const stage,
                           double const pdone,
                           double const etime)
    {
        if(memory.size() <= stage+1) {
            std::cout << "available memory exceeded. returning to previous stage" << std::endl;
        }

        // Create a sorted agenda of child states to expand
        memory[stage].expansion_map.clear();
        for(size_t i = 0; i < opc_subset.size(); ++i) {
            size_t const & pose_idx = opc_subset[i];
            double const & gain = memory[stage].detec_opts.gain[pose_idx];
            if(!memory[stage].detec_opts.detectables[pose_idx].empty()) {
                double duration = opc.getTravelTime(sequence[stage], pose_idx);
                double greedy_cost = duration / gain;
                double new_etime = etime + (1.0 - pdone) * duration;
                memory[stage].expansion_map.insert(std::make_pair(greedy_cost, std::make_pair(pose_idx, new_etime)));
            }
        }

        // Test for goal state
        if(memory[stage].expansion_map.empty()) {
            // This state has no children and is therefore a goal state.
            // Since we got here, this is the best goal state known yet.
            best_sequence = sequence;
            best_etime = etime;
            std::cout << "found new optimum with etime " << etime << std::endl;
            return;
        }

        // make sure next stage memory is initialized
        if(memory[stage+1].detec_opts.detectables.empty()) {
            memory[stage+1].detec_opts = memory[stage].detec_opts;
        }

        // Work through agenda
        double cutoff = memory[stage].expansion_map.begin()->first * 1.15;
        for(expansion_map_t::iterator it = memory[stage].expansion_map.begin();
            it != memory[stage].expansion_map.end();
            ++it)
        {
            if(it->first > cutoff) {
                // Vertical cutoff for branches that just seem too bad
                break;
            }
            size_t const & pose_idx = it->second.first;
            double const & new_etime = it->second.second;
            if(new_etime > best_etime) {
                // new_etime > best_etime => This branch is hopeless and can be skipped
                continue;
            }
            // Apply view pose and prepare memory for next stage
            sequence.push_back(pose_idx);
            memory[stage+1].detec_opts.assignEqualSize(memory[stage].detec_opts);
            memory[stage+1].detec_opts.removeCells(memory[stage].detec_opts.detectables[pose_idx], cgl);
            // Explore next stage
            makePlanRecursive(stage + 1, pdone + memory[stage].detec_opts.gain[pose_idx], new_etime);
            // Remove the explored pose
            sequence.pop_back();
            if(stage >= depth_limit) {
                // Beyond depth_limit, we are only interested in greedy solutions and therefore cut all alternatives.
                break;
            }
        }
    }
};

#endif // SEARCH_PLAN2_H
