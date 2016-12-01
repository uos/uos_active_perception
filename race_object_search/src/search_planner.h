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

    /** Get view pose sequence from last planning operation */
    std::vector<size_t> getSequence() { return best_sequence; }

    /** Get success probability from last planning operation */
    double getPdone() { return best_pdone; }

    /** Get expected run time from last planning operation */
    double getEtime() { return best_etime; }

    /** Returns true when branches were cut off during last search run */
    bool branchCutoffOccurred() { return branch_cutoff_occurred; }

    bool makePlan(size_t const arg_depth_limit,
                  double const arg_pdone_goal,
                  double const arg_max_rel_branch_cost,
                  long const timeout_msecs,
                  bool arg_use_domination,
                  double etime_bound = std::numeric_limits<double>::infinity())
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
        use_domination = arg_use_domination;
        if(memory.empty()) {
            memory.resize(1000);
        }
        memory[0].detec_opts.init(cgl, opc, opc_subset);
        sequence.clear();
        sequence.push_back(-1);
        best_sequence.clear();
        best_etime = etime_bound;
        branch_cutoff_occurred = false;

        return makePlanRecursive(0, 0.0, 0.0);
    }

    bool makeGreedy()
    {
        return makePlan(0, 1.0, 2.0, 0, false);
    }

    bool optimalOrder(size_t const arg_depth_limit,
                      double const arg_pdone_goal,
                      double const arg_max_rel_branch_cost,
                      long const timeout_msecs,
                      std::vector<size_t> const & input_sequence)
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
        use_domination = false;
        if(memory.empty()) {
            memory.resize(input_sequence.size());
        }
        memory[0].detec_opts.init(cgl, opc, opc_subset);
        sequence.clear();
        sequence.push_back(-1);
        best_sequence.clear();
        best_etime = std::numeric_limits<double>::infinity();
        branch_cutoff_occurred = false;

        std::cout << "Starting recursive optimal ordering..." << std::endl;
        return makePlanRecursive(0, 0.0, 0.0);
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

    struct ExpansionOption
    {
        size_t opc_subset_idx;
        double etime;
        double gain;
        double duration;
    };

    typedef std::multimap<double, ExpansionOption> expansion_map_t;

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
    double best_pdone;
    double max_rel_branch_cost;
    boost::posix_time::ptime timeout;
    bool use_domination;
    bool branch_cutoff_occurred;

    bool makePlanRecursive(size_t const stage,
                           double const pdone,
                           double const etime)
    {
        if(memory.size() <= stage+1) {
            std::cout << "available memory exceeded. returning to previous stage" << std::endl;
            return true;
        }

        // Test for timeout
        if(timeout != boost::date_time::not_a_date_time && timeout <= boost::posix_time::microsec_clock::local_time()) {
            return false;
        }

        // Test for goal state (pdone_goal condition)
        if(pdone > pdone_goal) {
            best_sequence = sequence;
            best_etime = etime;
            best_pdone = pdone;
            return true;
        }

        // Create a sorted agenda of child states to expand
        memory[stage].expansion_map.clear();
        for(size_t i = 0; i < opc_subset.size(); ++i) {
            size_t const & pose_idx = opc_subset[i];
            if(!memory[stage].detec_opts.detectables[i].empty()) {
                ExpansionOption expop;
                expop.opc_subset_idx = i;
                expop.gain = memory[stage].detec_opts.gain[i];
                expop.duration = opc.getTravelTime(sequence[stage], pose_idx);
                expop.etime = etime + (1.0 - pdone) * expop.duration;
                double greedy_cost = expop.duration / expop.gain;
                // Even if all poses are reachable from the start pose, some pairs of poses may be unconnected.
                // This is illogical and possibly due to map updates between subsequent calls of make_plan.
                // The easiest thing is to just filter these cases here.
                if(expop.duration < std::numeric_limits<double>::infinity())
                {
                  memory[stage].expansion_map.insert(std::make_pair(greedy_cost, expop));
                }
            }
        }

        // Test for goal state (nothing left to explore condition)
        if(memory[stage].expansion_map.empty()) {
            best_sequence = sequence;
            best_etime = etime;
            best_pdone = pdone;
            return true;
        }

        // make sure next stage memory is properly initialized
        if(memory[stage].detec_opts.detectables.size() != memory[stage+1].detec_opts.detectables.size()) {
            memory[stage+1].detec_opts = memory[stage].detec_opts;
        }

        // Work through agenda
        std::vector<const ExpansionOption*> potentially_dominating;
        potentially_dominating.reserve(memory[stage].expansion_map.size());
        double cutoff = memory[stage].expansion_map.begin()->first * max_rel_branch_cost;
        for(typename expansion_map_t::iterator it = memory[stage].expansion_map.begin();
            it != memory[stage].expansion_map.end();
            ++it)
        {
            if(it->first > cutoff) {
                // cutoff for branches that just seem too bad
                branch_cutoff_occurred = true;
                break;
            }
            ExpansionOption const & expop = it->second;
            if(expop.etime > best_etime) {
                // new_etime > best_etime => This branch is hopeless and can be skipped
                continue;
            }

            // Check for strict domination
            bool found_dominating = false;
            for(size_t dom_idx = 0; !found_dominating && dom_idx < potentially_dominating.size(); ++dom_idx)
            {
                if(potentially_dominating[dom_idx]->gain > expop.gain &&
                   potentially_dominating[dom_idx]->duration < expop.duration)
                {
                    found_dominating = true;
                }
            }
            if(found_dominating) continue;
            if(use_domination) potentially_dominating.push_back(&expop);

            // Apply view pose and prepare memory for next stage
            sequence.push_back(opc_subset[expop.opc_subset_idx]);
            memory[stage+1].detec_opts.assignEqualSize(memory[stage].detec_opts);
            memory[stage+1].detec_opts.removeCells(memory[stage].detec_opts.detectables[expop.opc_subset_idx], cgl);
            // Explore next stage
            bool not_timed_out = makePlanRecursive(stage + 1,
                                    pdone + (1.0 - pdone) * memory[stage].detec_opts.gain[expop.opc_subset_idx],
                                    expop.etime);
            if(!not_timed_out) return false;
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
