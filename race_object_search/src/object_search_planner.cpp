#include "object_search_planner.h"

#include "observation_pose_collection.h"
#include "search_plan.h"
#include "search_planner.h"

#include <visualization_msgs/Marker.h>
#include <uos_active_perception_msgs/GetObservationCameraPoses.h>
#include <boost/date_time/posix_time/posix_time.hpp>

ObjectSearchPlanner::ObjectSearchPlanner()
:
    m_node_handle("~"),
    m_node_handle_pub(),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/search_plan_marker", 10000)),
    m_observe_volumes_server(m_node_handle_pub,
                             "observe_volumes",
                             boost::bind(&ObjectSearchPlanner::observeVolumesCb, this, _1),
                             false),
    m_agent(m_tf_listener, m_world_frame_id)
{
    m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));

    m_observe_volumes_server.start();
}

void ObjectSearchPlanner::observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal_ptr)
{
    const double HORIZON = 600; // [s]

    ROS_INFO("Got a new goal!");
    race_object_search::ObserveVolumesGoal const & goal = *goal_ptr.get();

    // get current robot pose
    const tf::Pose robot_pose = m_agent.getCurrentRobotPose();
    const tf::Pose cam_pose = m_agent.getCurrentCamPose();

    ROS_INFO("retrieving pose candidates");
    ObservationPoseCollection opc;
    size_t n_cells = 0;
    {
        uos_active_perception_msgs::GetObservationCameraPoses pose_candidates_call;
        pose_candidates_call.request.sample_size = 200;
        pose_candidates_call.request.ray_skip = 0.75;
        pose_candidates_call.request.roi = goal.roi;
        if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
        {
            ROS_ERROR("service call failed");
            m_observe_volumes_server.setAborted();
            return;
        }
        opc.addPoses(pose_candidates_call.response.camera_poses,
                     pose_candidates_call.response.target_points,
                     pose_candidates_call.response.cvms,
                     pose_candidates_call.response.object_sets);

        for(size_t i = 0; i < pose_candidates_call.response.roi_cell_counts.size(); ++i) {
            n_cells += pose_candidates_call.response.roi_cell_counts[i];
        }
    }

    ROS_INFO("building travel time lookup tables");
    opc.prepareInitialTravelTimeLut(m_agent, robot_pose, cam_pose, m_world_frame_id);
    opc.pruneUnreachablePoses();
    opc.prepareTravelTimeLut(m_agent, m_world_frame_id);

    ROS_INFO("Preparing greedy strategy");

    EqualProbabilityCellGain epcg;
    {
        // HACK to find the number of discoverable cells (nr of cells seen by greedy strategy)
        epcg.p = 1.0 / n_cells;
        SearchPlan<EqualProbabilityCellGain> sp(epcg, opc);
        sp.greedy(HORIZON);
        epcg.p = 1.0 / sp.detected_cells[sp.last_idx].size();
    }

    SearchPlanner<EqualProbabilityCellGain> spl(epcg, opc);
    SearchPlan<EqualProbabilityCellGain> sp(epcg, opc);

    std::vector<size_t> result_seq;
    double result_etime;
    bool success;

    // GREEDY ---

    boost::posix_time::ptime tick = boost::posix_time::microsec_clock::local_time();
    success = spl.makeGreedy(result_seq, result_etime);
    boost::posix_time::ptime now  = boost::posix_time::microsec_clock::local_time();
    std::cout << "greedy took msec: " << (now-tick).total_milliseconds() << std::endl;
    if(success) {
        sp.pushSequence(result_seq, 1, result_seq.size());
        ROS_INFO_STREAM("Greedy plan has " << sp.time.size() << " steps and an etime of " << sp.etime[sp.last_idx]);
        ROS_INFO_STREAM("Greedy plan has " << result_seq.size() << " steps and an etime of " << result_etime);
        sp.writeTimeplot("timeplot_greedy.tab");
    } else {
        ROS_INFO("Greedy plan failed!");
    }

    // GREEDY-REORDER ---

    spl.optimalOrder(result_seq.size(), 1.0, std::numeric_limits<double>::infinity(), 0, result_seq, result_seq, result_etime);
    sp.clear();
    sp.pushSequence(result_seq, 1, result_seq.size());
    ROS_INFO_STREAM("Greedy-Reordered plan has " << sp.time.size() << " steps and an etime of " << sp.etime[sp.last_idx]);
    ROS_INFO_STREAM("Greedy-Reordered plan has " << result_seq.size() << " steps and an etime of " << result_etime);
    sp.writeTimeplot("timeplot_greedy_reordered.tab");

    // OPTIMAL ---

    ROS_INFO("Starting search");
    boost::posix_time::ptime tick2 = boost::posix_time::microsec_clock::local_time();
    spl.makePlan(5, 1.0, 1.5, 0, result_seq, result_etime);
    boost::posix_time::ptime now2  = boost::posix_time::microsec_clock::local_time();
    ROS_INFO_STREAM("Search took msec: " << (now2-tick2).total_milliseconds());
    sp.clear();
    sp.pushSequence(result_seq, 1, result_seq.size());
    sp.sendMarker(m_world_frame_id, m_marker_pub, "optimal_plan");
    ROS_INFO_STREAM("etime " << sp.etime[sp.last_idx]);
    sp.writeTimeplot("timeplot_optimal.tab");

    ROS_INFO("Done");
    m_observe_volumes_server.setSucceeded();
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_search_planner");
    ObjectSearchPlanner node;
    ROS_INFO("object_search_planner: Initialized!");
    ros::spin();
}
