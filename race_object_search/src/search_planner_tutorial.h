#ifndef SEARCH_PLANNER_TUTORIAL_H
#define SEARCH_PLANNER_TUTORIAL_H

#include "observation_pose_collection.h"

#include <boost/date_time/posix_time/ptime.hpp>

#include <vector>
#include <iostream>
#include <map>


template <class CELL_GAIN_LOOKUP>
class SearchPlannerTutorial
{
public:
  /**
     * @brief SearchPlannerTutorial
     *
     * @param cgl The Cell Gain Lookup maps each cell ID to an information gain for that cell.
     *
     * @param opc The Observation Pose Collection holds the observation poses, the cells
     *            observed by each and the expected travel times between each pair of poses.
     *
     * @param arg_depth_limit          do depth-first search up to level `depth_limit` (default: 2)
     *
     * @param arg_pdone_goal           stop when remaining probability of finding the object is < `pdone_goal`
     *
     * @param arg_max_rel_branch_cost  restrict search to poses that are not worse than `max_rel_branch_cost`
     *                                 (default: 1.3) compared to the greedy option
     *
     * @param timeout_msecs            time limit for search
     */
  SearchPlannerTutorial(CELL_GAIN_LOOKUP const & cgl,
                     ObservationPoseCollection const & opc,
                     size_t const arg_depth_limit,
                     double const arg_pdone_goal,
                     double const arg_max_rel_branch_cost,
                     long const timeout_msecs)
    : cgl(cgl)
    , opc(opc)
    , depth_limit(arg_depth_limit)
    , pdone_goal(arg_pdone_goal)
    , max_rel_branch_cost(arg_max_rel_branch_cost)
    , memory(MAX_RECURSION_DEPTH)
    , best_etime(std::numeric_limits<double>::infinity())
  {
    if (timeout_msecs > 0)
    {
      timeout = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(timeout_msecs);
    }
    else
    {
      timeout = boost::date_time::not_a_date_time;
    }
    memory[0].detec_opts.init(cgl, opc);
    sequence.push_back(-1);
  }

  /** Get view pose sequence from last planning operation */
  std::vector<size_t> getSequence()
  {
    return best_sequence;
  }

  /** Get expected run time from last planning operation */
  double getEtime()
  {
    return best_etime;
  }

  /** Make a plan and fill best_sequence + best_etime. Will be called exactly once;
   *  afterwards, this object will be destroyed. */
  bool makePlan()
  {
    // TODO: m_already_run = true
    return makePlanRecursive(0, 0.0, 0.0);
  }

private:
  static const int MAX_RECURSION_DEPTH = 1000;

  typedef geometry::detection_t detection_t;

  /**
   * @brief The DetectionOptions class holds all observation poses, updated to the current
   * search state (i.e., all cells that have already been observed by other poses in the
   * sequence so far have been removed).
   */
  class DetectionOptions
  {
  public:
    std::vector<detection_t> detectables;
    std::vector<double> gain;

    void init(CELL_GAIN_LOOKUP const & cgl,
              ObservationPoseCollection const & opc)
    {
      detectables.clear();
      detectables.reserve(opc.getPoses().size());
      gain.clear();
      gain.reserve(opc.getPoses().size());
      for (size_t i = 0; i < opc.getPoses().size(); ++i)
      {
        detection_t const & d = opc.getPoses()[i].cell_id_sets[0];
        detectables.push_back(d);
        double g = 1.0;
        for (detection_t::iterator it = d.begin(); it != d.end(); ++it)
        {
          g *= 1.0 - cgl(*it);
        }
        gain.push_back(1.0 - g);
      }
    }

    DetectionOptions & assignEqualSize(const DetectionOptions & other)
    {
      assert(detectables.size() == other.detectables.size());
      for (size_t i = 0; i < detectables.size(); ++i)
      {
        detectables[i].clear();
        detectables[i].insert(other.detectables[i].begin(), other.detectables[i].end());
        gain[i] = other.gain[i];
      }
      return *this;
    }

    void removeCells(detection_t const & d, CELL_GAIN_LOOKUP const & cgl)
    {
      for (size_t i = 0; i < detectables.size(); ++i)
      {
        double p_not_in_removed = 1.0;
        for (detection_t::iterator it = d.begin(); it != d.end(); ++it)
        {
          if (detectables[i].erase(*it))
          {
            p_not_in_removed *= 1.0 - cgl(*it);
          }
        }
        // update gain
        gain[i] = 1.0 - (1.0 - gain[i]) / p_not_in_removed;
      }
    }
  };

  /**
   * @brief The ExpansionOption struct holds the information about an "expansion option",
   * i.e., one observation pose that could be chosen as a next step in the search.
   */
  struct ExpansionOption
  {
    size_t opc_idx;   // index in `opc` to which this observation pose corresponds
    double etime;     // expected time to find object after going to all poses in sequence and then this one
    double gain;      // expected information gain of just this pose
    double duration;  // expected travel time between the current pose (the last one in the sequence so far) and this pose
  };

  /**
   * @brief A map of `<greedy_cost, expop>` pairs, where `greedy_cost = expop.duration / expop.gain`.
   * The iterator is sorted, so that the expansion option with the lowest greedy cost is returned first.
   */
  typedef std::multimap<double, ExpansionOption> expansion_map_t;

  struct SearchStateMemory
  {
    DetectionOptions detec_opts;
    expansion_map_t expansion_map;
  };

  CELL_GAIN_LOOKUP const & cgl;           // see constructor comment
  ObservationPoseCollection const & opc;  // see constructor comment
  std::vector<SearchStateMemory> memory;  // cache to speed up search
  std::vector<size_t> sequence;           // sequence of indexes to `opc` representing the current path in the search

  std::vector<size_t> best_sequence;      // best sequence found so far
  double best_etime;                      // expected time to find object for `best_sequence`
  double best_pdone;                      // remaining probability that the target object could be found
                                          // in the still unexplored cells after executing `best_sequence`

  size_t depth_limit;                     // see constructor comment
  double pdone_goal;                      // see constructor comment
  double max_rel_branch_cost;             // see constructor comment
  boost::posix_time::ptime timeout;       // see constructor comment

  /**
   * @brief makePlanRecursive
   *
   * @param cur_depth current search depth (root of search tree = 0)
   *
   * @param pdone     remaining probability that the target object can be found in
   *                  the still unexplored cells after executing the `sequence` so far
   *
   * @param etime     expected time to find object after executing the `sequence` so far
   *
   * @return          `false` if timed out, `true` otherwise
   */
  bool makePlanRecursive(size_t const cur_depth,
                         double const pdone,
                         double const etime)
  {
    if (cur_depth >= MAX_RECURSION_DEPTH)
    {
      std::cout << "Maximum recursion depth exceeded. Returning to previous stage." << std::endl;
      return true;
    }

    // Test for timeout
    if (timeout != boost::date_time::not_a_date_time
        && timeout <= boost::posix_time::microsec_clock::local_time())
    {
      return false;
    }

    // Test for goal state (pdone_goal condition)
    if (pdone > pdone_goal)
    {
      best_sequence = sequence;
      best_etime = etime;
      best_pdone = pdone;
      return true;
    }

    // Create a sorted agenda of child states to expand
    memory[cur_depth].expansion_map.clear();
    for (size_t i = 0; i < opc.getPoses().size(); ++i)
    {
      if (!memory[cur_depth].detec_opts.detectables[i].empty())
      {
        ExpansionOption expop;
        expop.opc_idx = i;
        expop.gain = memory[cur_depth].detec_opts.gain[i];
        expop.duration = opc.getTravelTime(sequence[cur_depth], i);
        expop.etime = etime + (1.0 - pdone) * expop.duration;
        double greedy_cost = expop.duration / expop.gain;

        // Even if all poses are reachable from the start pose, some pairs of poses may be unconnected.
        // This is illogical and possibly due to map updates between subsequent calls of make_plan.
        // The easiest thing is to just filter these cases here.
        if (expop.duration < std::numeric_limits<double>::infinity())
        {
          memory[cur_depth].expansion_map.insert(std::make_pair(greedy_cost, expop));
        }
      }
    }

    // Test for goal state (nothing left to explore condition)
    if (memory[cur_depth].expansion_map.empty())
    {
      best_sequence = sequence;
      best_etime = etime;
      best_pdone = pdone;
      return true;
    }

    // make sure next stage memory is properly initialized
    if (memory[cur_depth].detec_opts.detectables.size() != memory[cur_depth + 1].detec_opts.detectables.size())
    {
      memory[cur_depth + 1].detec_opts = memory[cur_depth].detec_opts;
    }

    /* **************************** BEGIN: EDIT BETWEEN THESE LINES ****************************
     *
     * Feel free to edit anything else in this class or even anywhere else in this package,
     * but you can implement a new search strategy by changing this part, so this is a good start.
     *
     * The code between these lines implements a simplified version of the main search strategy
     * of FLAP4CAOS (`planning_mode = "search"`). Your job is to replace it by something different.
     *
     * Feel free to ignore some parameters that are specific to this search strategy:
     *
     * - depth_limit
     * - max_rel_branch_cost
     *
     * Also feel free to do something completely different than depth-first search. Play around and be brave!
     */

    // Work through sorted agenda
    double cutoff = memory[cur_depth].expansion_map.begin()->first * max_rel_branch_cost;
    for (typename expansion_map_t::iterator it = memory[cur_depth].expansion_map.begin();
         it != memory[cur_depth].expansion_map.end();
         ++it)
    {
      if (it->first > cutoff)
      {
        // cutoff for branches that just seem too bad
        break;
      }
      ExpansionOption const & expop = it->second;
      if (expop.etime > best_etime)
      {
        // new_etime > best_etime => This branch is hopeless and can be skipped
        continue;
      }

      // Apply view pose and prepare memory for next stage
      sequence.push_back(expop.opc_idx);
      memory[cur_depth + 1].detec_opts.assignEqualSize(memory[cur_depth].detec_opts);
      memory[cur_depth + 1].detec_opts.removeCells(memory[cur_depth].detec_opts.detectables[expop.opc_idx], cgl);

      // Explore next stage
      bool not_timed_out = makePlanRecursive(cur_depth + 1,
                                             pdone + (1.0 - pdone) * memory[cur_depth].detec_opts.gain[expop.opc_idx],
                                             expop.etime);
      if (!not_timed_out) return false;

      // Remove the explored pose
      sequence.pop_back();
      if (cur_depth >= depth_limit)
      {
        // Beyond depth_limit, we are only interested in greedy solutions and therefore cut all alternatives.
        break;
      }
    }
    /* **************************** END: EDIT BETWEEN THESE LINES **************************** */

    return true;
  }
};

#endif // SEARCH_PLANNER_TUTORIAL_H
