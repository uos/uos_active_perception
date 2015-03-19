#ifndef SEARCH_PLANNER_H
#define SEARCH_PLANNER_H

#include "observation_pose_collection.h"

#include <visualization_msgs/Marker.h>
#include <boost/date_time/posix_time/ptime.hpp>

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
                  double const arg_pdone_goal,
                  double const arg_max_rel_branch_cost,
                  long const timeout_msecs,
                  std::vector<size_t> & result_sequence,
                  double & result_etime)
    {
        opc_subset.resize(opc.getPoses().size());
        for(size_t i = 0; i < opc_subset.size(); ++i) {
            opc_subset[i] = i;
        }
        depth_limit = arg_depth_limit;
        pdone_goal = arg_pdone_goal;
        max_rel_branch_cost = arg_max_rel_branch_cost;
        if(timeout_msecs > 0) {
            timeout = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(timeout_msecs);
        } else {
            timeout = boost::date_time::not_a_date_time;
        }
        if(memory.empty()) {
            memory.resize(1000);
        }
        memory[0].detec_opts.init(cgl, opc, opc_subset);
        sequence.clear();
        sequence.push_back(-1);
        best_sequence.clear();
        best_etime = std::numeric_limits<double>::infinity();

        std::cout << "Starting recursive search..." << std::endl;
        bool finished = makePlanRecursive(0, 0.0, 0.0);

        result_sequence = best_sequence;
        result_etime = best_etime;
        return finished;
    }

    bool makeGreedy(std::vector<size_t> & result_sequence,
                    double & result_etime)
    {
        return makePlan(0, 1.0, 2.0, 0, result_sequence, result_etime);
    }

    bool optimalOrder(size_t const arg_depth_limit,
                      double const arg_pdone_goal,
                      double const arg_max_rel_branch_cost,
                      long const timeout_msecs,
                      std::vector<size_t> const & input_sequence,
                      std::vector<size_t> & result_sequence,
                      double & result_etime)
    {
        opc_subset.clear();
        opc_subset.insert(opc_subset.end(), input_sequence.begin() + 1, input_sequence.end());
        depth_limit = arg_depth_limit;
        pdone_goal = arg_pdone_goal;
        max_rel_branch_cost = arg_max_rel_branch_cost;
        if(timeout_msecs > 0) {
            timeout = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(timeout_msecs);
        } else {
            timeout = boost::date_time::not_a_date_time;
        }
        if(memory.empty()) {
            memory.resize(input_sequence.size());
        }
        memory[0].detec_opts.init(cgl, opc, opc_subset);
        sequence.clear();
        sequence.push_back(-1);
        best_sequence.clear();
        best_etime = std::numeric_limits<double>::infinity();

        std::cout << "Starting recursive optimal ordering..." << std::endl;
        bool finished = makePlanRecursive(0, 0.0, 0.0);

        result_sequence = best_sequence;
        result_etime = best_etime;
        return finished;
    }

private:
    typedef geometry::detection_t detection_t;
    class DetectionOptions
    {
    public:
        std::vector<detection_t> detectables;
        std::vector<double> gain;

        void init(CELL_GAIN_LOOKUP const & cgl,
                  ObservationPoseCollection const & opc,
                  std::vector<size_t> const & opc_subset)
        {
            detectables.clear();
            detectables.reserve(opc_subset.size());
            gain.clear();
            gain.reserve(opc_subset.size());
            for(size_t i = 0; i < opc_subset.size(); ++i) {
                detection_t const & d = opc.getPoses()[opc_subset[i]].cell_id_sets[0];
                detectables.push_back(d);
                double g = 1.0;
                for(detection_t::iterator it = d.begin(); it != d.end(); ++it) {
                    g *= 1.0 - cgl(*it);
                }
                gain.push_back(1.0 - g);
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
                double p_not_in_removed = 1.0;
                for(detection_t::iterator it = d.begin(); it != d.end(); ++it) {
                    if(detectables[i].erase(*it)) {
                        p_not_in_removed *= 1.0 - cgl(*it);
                    }
                }
                // update gain
                gain[i] = 1.0 - (1.0 - gain[i]) / p_not_in_removed;
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
    double pdone_goal;
    std::vector<SearchStateMemory> memory;
    std::vector<size_t> sequence;
    std::vector<size_t> best_sequence;
    double best_etime;
    double max_rel_branch_cost;
    boost::posix_time::ptime timeout;

    bool makePlanRecursive(size_t const stage,
                           double const pdone,
                           double const etime)
    {
        if(memory.size() <= stage+1) {
            std::cout << "available memory exceeded. returning to previous stage" << std::endl;
            return true;
        }

        // Test for goal state (pdone_goal condition)
        if(pdone >= pdone_goal) {
            best_sequence = sequence;
            best_etime = etime;
            std::cout << "found new optimum with etime " << etime << std::endl;
            return true;
        }

        // Create a sorted agenda of child states to expand
        memory[stage].expansion_map.clear();
        for(size_t i = 0; i < opc_subset.size(); ++i) {
            size_t const & pose_idx = opc_subset[i];
            if(!memory[stage].detec_opts.detectables[i].empty()) {
                double const & gain = memory[stage].detec_opts.gain[i];
                double duration = opc.getTravelTime(sequence[stage], pose_idx);
                double greedy_cost = duration / gain;
                double new_etime = etime + (1.0 - pdone) * duration;
                // Even if all poses are reachable from the start pose, some pairs of poses may be unconnected.
                // This is illogical and possibly due to map updates between subsequent calls of make_plan.
                // The easiest thing is to just filter these cases here.
                assert(duration < std::numeric_limits<double>::infinity());
                memory[stage].expansion_map.insert(std::make_pair(greedy_cost, std::make_pair(i, new_etime)));
            }
        }

        // Test for goal state (nothing left to explore condition)
        if(memory[stage].expansion_map.empty()) {
            best_sequence = sequence;
            best_etime = etime;
            std::cout << "found new optimum with etime " << etime << std::endl;
            return true;
        }

        // make sure next stage memory is properly initialized
        if(memory[stage].detec_opts.detectables.size() != memory[stage+1].detec_opts.detectables.size()) {
            memory[stage+1].detec_opts = memory[stage].detec_opts;
        }

        // Work through agenda
        double cutoff = memory[stage].expansion_map.begin()->first * max_rel_branch_cost;
        for(expansion_map_t::iterator it = memory[stage].expansion_map.begin();
            it != memory[stage].expansion_map.end();
            ++it)
        {
            if(it->first > cutoff) {
                // cutoff for branches that just seem too bad
                break;
            }
            size_t const & opc_subset_idx = it->second.first;
            double const & new_etime = it->second.second;
            if(new_etime > best_etime) {
                // new_etime > best_etime => This branch is hopeless and can be skipped
                continue;
            }
            // Apply view pose and prepare memory for next stage
            sequence.push_back(opc_subset[opc_subset_idx]);
            memory[stage+1].detec_opts.assignEqualSize(memory[stage].detec_opts);
            memory[stage+1].detec_opts.removeCells(memory[stage].detec_opts.detectables[opc_subset_idx], cgl);
            // Explore next stage
            makePlanRecursive(stage + 1,
                              pdone + (1.0 - pdone) * memory[stage].detec_opts.gain[opc_subset_idx],
                              new_etime);
            if(timeout != boost::date_time::not_a_date_time &&
               boost::posix_time::microsec_clock::local_time() > timeout) {
                return false;
            }
            // Remove the explored pose
            sequence.pop_back();
            if(stage >= depth_limit) {
                // Beyond depth_limit, we are only interested in greedy solutions and therefore cut all alternatives.
                break;
            }
        }

        return true;
    }
};

#endif // SEARCH_PLANNER_H
