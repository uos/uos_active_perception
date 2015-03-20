#ifndef OBJECT_SEARCH_MANAGER_H
#define OBJECT_SEARCH_MANAGER_H

#include "observation_pose_collection.h"
#include "search_plan.h"
#include "search_planner.h"
#include "ros_serialization_helper.h"
#include "geometry.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <race_object_search/ObserveVolumesAction.h>
#include <visualization_msgs/Marker.h>
#include <uos_active_perception_msgs/GetObservationCameraPoses.h>
#include <uos_active_perception_msgs/EvaluateObservationCameraPoses.h>
#include <uos_active_perception_msgs/GetBboxOccupancy.h>

#include <fstream>

template<class TAgent>
class ObjectSearchManager
{
public:
    ObjectSearchManager()
    :
        m_node_handle("~"),
        m_node_handle_pub(),
        m_tf_listener(),
        m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/object_search_marker", 10000)),
        m_map_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/next_best_view_node/marker_in", 10000)),
        m_observe_volumes_server(m_node_handle_pub,
                                 "observe_volumes",
                                 boost::bind(&ObjectSearchManager::observeVolumesCb, this, _1),
                                 false),
        m_agent(m_tf_listener, m_world_frame_id)
    {
        m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));
        m_node_handle.param("log_dir", m_log_dir, std::string(""));
        m_node_handle.param("persistent_sample_dir", m_ps_dir, std::string(""));
        // sampling params
        m_node_handle.param("local_sample_size", m_local_sample_size, 100);
        m_node_handle.param("global_sample_size", m_global_sample_size, 200);
        m_node_handle.param("ray_skip", m_ray_skip, 0.75);
        // planning params
        m_node_handle.param("planning_mode", m_planning_mode, std::string("simple"));
        m_node_handle.param("depth_limit", m_depth_limit, 5);
        m_node_handle.param("relative_lookahead", m_relative_lookahead, 1.0);
        m_node_handle.param("max_rel_branch_cost", m_max_rel_branch_cost, 1.8);
        m_node_handle.param("planning_timeout", m_planning_timeout, 20.0);
        m_node_handle.param("keep_planned_poses", m_keep_planned_poses, true);

        m_use_static_poses = !m_ps_dir.empty();
        if(m_use_static_poses) m_keep_planned_poses = false;

        if(!m_log_dir.empty()) {
            std::stringstream fname;
            fname << m_log_dir << "/vals.log";
            std::ifstream fvalsin(fname.str().c_str(), std::ios::binary);
            fvalsin.seekg(0, std::ios::end);
            if(fvalsin.good() && fvalsin.tellg() > 0) {
                fvalsin.close();
                ROS_ERROR_STREAM("File " << fname.str() << " exists and is not empty. Will not write any logs!");
                m_log_dir = "";
            } else {
                fvalsin.close();
                // open logging and evaluation data files
                fname.str("");
                fname.clear();
                fname << m_log_dir << "/vals.log";
                m_file_vals.open(fname.str().c_str(), std::ofstream::trunc);
                fname.str("");
                fname.clear();
                fname << m_log_dir << "/events.log";
                m_file_events.open(fname.str().c_str(), std::ofstream::trunc);

                // dump node params
                fname.str("");
                fname.clear();
                fname << m_log_dir << "/params.log";
                std::ofstream fparams(fname.str().c_str(), std::ofstream::trunc);
                fparams << "local_sample_size\t" << m_local_sample_size << std::endl;
                fparams << "global_sample_size\t" << m_global_sample_size << std::endl;
                fparams << "ray_skip\t" << m_ray_skip << std::endl;
                fparams << "planning_mode\t" << m_planning_mode << std::endl;
                fparams << "depth_limit\t" << m_depth_limit << std::endl;
                fparams << "relative_lookahead\t" << m_relative_lookahead << std::endl;
                fparams << "max_rel_branch_cost\t" << m_max_rel_branch_cost << std::endl;
                fparams << "planning_timeout\t" << m_planning_timeout << std::endl;
            }
        }

        m_observe_volumes_server.start();
        ROS_INFO_STREAM(ros::this_node::getName() << ": Initialized!");
    }

private:
    ros::NodeHandle m_node_handle, m_node_handle_pub;
    tf::TransformListener m_tf_listener;
    ros::Publisher m_marker_pub, m_map_marker_pub;
    actionlib::SimpleActionServer<race_object_search::ObserveVolumesAction> m_observe_volumes_server;
    TAgent m_agent;
    std::string m_world_frame_id;
    // sampling params
    int m_local_sample_size;
    int m_global_sample_size;
    double m_ray_skip;
    bool m_use_static_poses;
    bool m_keep_planned_poses;
    // planning params
    std::string m_planning_mode;
    int m_depth_limit;
    double m_relative_lookahead;
    double m_max_rel_branch_cost;
    double m_planning_timeout;

    // logging and evaluation data files
    std::string m_ps_dir;
    std::string m_log_dir;
    std::ofstream m_file_events;
    std::ofstream m_file_vals;

    struct EqualProbabilityCellGain
    {
         double p;
         double operator ()(uint64_t const & cell) const
         {
             return p;
         }
    };

    struct RegionalProbabilityCellGain
    {
         geometry::MapRegionCollection map_region_collection;
         std::vector<double> cell_gains;

         double operator ()(uint64_t const & cell) const
         {
             size_t idx = map_region_collection.findRegion(geometry::cellIdIntToMsg(cell));
             if(idx > cell_gains.size())
             {
                 ROS_WARN("Encountered cell that does not belong to any region");
                 return 0.0;
             }
             return cell_gains[idx];
         }
    };

    void logerror(const std::string & str)
    {
        ROS_ERROR_STREAM(str.c_str());
        if(m_file_events.is_open()) {
            m_file_events << ros::WallTime::now() << "\t" << str << std::endl;
        }
    }

    void loginfo(const std::string & str)
    {
        ROS_INFO_STREAM(str.c_str());
        if(m_file_events.is_open()) {
            m_file_events << ros::WallTime::now() << "\t" << str << std::endl;
        }
    }

    void logtime(const std::string & what, ros::WallTime t0)
    {
        ros::WallDuration time = ros::WallTime::now() - t0;
        ROS_INFO_STREAM("value of " << what << ": " << time);
        if(m_file_vals.is_open()) {
            m_file_vals << what << "\t" << time << std::endl;
        }
    }

    void logval(const std::string & what, double val)
    {
        ROS_INFO_STREAM("value of " << what << ": " << val);
        if(m_file_vals.is_open()) {
            m_file_vals << what << "\t" << val << std::endl;
        }
    }

    void pubMarker(const geometry::detection_t & cells, const std::string & ns, double r, double g, double b, double a)
    {
        static size_t id = 0;

        visualization_msgs::Marker marker;
        marker.ns = ns;
        marker.id = id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.lifetime = ros::Duration(10.0);
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        for(geometry::detection_t::iterator it = cells.begin(); it != cells.end(); ++it)
        {
            uos_active_perception_msgs::CellId cellid = geometry::cellIdIntToMsg(*it);
            geometry_msgs::Point p;
            p.x = cellid.x;
            p.y = cellid.y;
            p.z = cellid.z;
            marker.points.push_back(p);
        }
        m_map_marker_pub.publish(marker);
    }

    // Callbacks
    void observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal_ptr)
    {
        using namespace geometry;

        ROS_INFO("Got a new goal!");
        size_t iteration_counter = 0;
        std::stringstream fname;
        ros::WallTime t0;
        size_t marker_count = 0;
        race_object_search::ObserveVolumesGoal const & goal = *goal_ptr.get();

        // persistence buffers
        std::vector<geometry_msgs::Pose> persistent_samples;
        ObservationPoseCollection opc;
        if(!m_ps_dir.empty())
        {
            persistent_samples = ObservationPoseCollection::deserializePoses(m_ps_dir);
        }

        // Unknown cell tracking
        std::vector<detection_t> unknown_roi_cell_ids(goal.roi.size());
        detection_t expected_view;

        // Construct regional probability cell gain
        RegionalProbabilityCellGain rpcg;
        {
            std::vector<double> roi_cell_gain(goal.roi.size());
            MapRegionCollection region_collection;
            region_collection.regions.resize(goal.roi.size());
            for(size_t i = 0; i < goal.roi.size(); ++i)
            {
                uos_active_perception_msgs::GetBboxOccupancy get_bbox_occupancy;
                get_bbox_occupancy.request.bbox = goal.roi[i];
                while(!ros::service::call("/get_bbox_occupancy", get_bbox_occupancy))
                {
                    logerror("get_bbox_occupancy service call failed, will try again");
                    ros::WallDuration(5).sleep();
                }
                region_collection.regions[i].min = get_bbox_occupancy.response.bbox_min;
                region_collection.regions[i].max = get_bbox_occupancy.response.bbox_max;
                roi_cell_gain[i] = 1.0 - std::pow(1.0 - goal.p[i], 1.0 / region_collection.regions[i].cellCount());
            }
            rpcg.map_region_collection = region_collection;
            rpcg.cell_gains = roi_cell_gain;
        }


        while(ros::ok())
        {
            if(m_observe_volumes_server.isPreemptRequested())
            {
                ROS_INFO("Preempted!");
                m_observe_volumes_server.setPreempted();
                return;
            }

            // evaluate unknown cell difference to last iteration
            detection_t unexpected_cells;
            double igain = 1.0, iprobability_sum = 1.0;
            for(size_t i = 0; i < goal.roi.size(); ++i)
            {
                uos_active_perception_msgs::GetBboxOccupancy get_bbox_occupancy;
                get_bbox_occupancy.request.bbox = goal.roi[i];
                while(!ros::service::call("/get_bbox_occupancy", get_bbox_occupancy))
                {
                    logerror("get_bbox_occupancy service call failed, will try again");
                    ros::WallDuration(5).sleep();
                }
                iprobability_sum *= std::pow(1.0 - rpcg.cell_gains[i],
                                             get_bbox_occupancy.response.unknown.cell_ids.size());
                igain *= std::pow(1.0 - rpcg.cell_gains[i],
                                  unknown_roi_cell_ids[i].size() - get_bbox_occupancy.response.unknown.cell_ids.size());

                detection_t unknown_cell_ids = cellIdsMsgToDetection(get_bbox_occupancy.response.unknown);
                for(detection_t::iterator it = unknown_roi_cell_ids[i].begin();
                    it != unknown_roi_cell_ids[i].end();
                    ++it)
                {
                    if(!unknown_cell_ids.count(*it) && !expected_view.erase(*it)) unexpected_cells.insert(*it);
                }
                unknown_roi_cell_ids[i] = unknown_cell_ids;
            }
            double gain = 1.0 - igain;
            double probability_sum = 1.0 - iprobability_sum;
            if(iteration_counter > 0)
            {
                logval("gain", gain);
            }
            pubMarker(expected_view, "missed_cells", 1.0, 0.0, 1.0, 0.5);
            pubMarker(unexpected_cells, "unexpected_cells", 0.0, 1.0, 0.0, 0.5);

            iteration_counter++;
            m_file_events << iteration_counter << std::endl;
            m_file_vals << iteration_counter << std::endl;

            logval("probability_sum", probability_sum);

            // get current robot pose
            const tf::Pose robot_pose = m_agent.getCurrentRobotPose();
            const tf::Pose cam_pose = m_agent.getCurrentCamPose();

            t0 = ros::WallTime::now();
            if(!persistent_samples.empty())
            {
                ROS_INFO("reevaluating pose candidates");
                {
                    uos_active_perception_msgs::EvaluateObservationCameraPoses pose_candidates_call;
                    pose_candidates_call.request.camera_poses = persistent_samples;
                    pose_candidates_call.request.ray_skip = m_ray_skip;
                    pose_candidates_call.request.roi = goal.roi;
                    while(!ros::service::call("/evaluate_observation_camera_poses", pose_candidates_call))
                    {
                        logerror("evaluate_observation_camera_poses service call failed");
                        ros::Duration(5).sleep();
                    }
                    opc.addPoses(pose_candidates_call.response.camera_poses,
                                 pose_candidates_call.response.target_points,
                                 pose_candidates_call.response.cvms,
                                 pose_candidates_call.response.object_sets);
                }
            }
            if(!m_use_static_poses || (persistent_samples.empty() && iteration_counter == 1))
            {
                opc = ObservationPoseCollection();
                ROS_INFO("retrieving local pose candidates");
                {
                    uos_active_perception_msgs::GetObservationCameraPoses pose_candidates_call;
                    pose_candidates_call.request.sample_size = m_local_sample_size;
                    pose_candidates_call.request.ray_skip = m_ray_skip;
                    pose_candidates_call.request.roi = goal.roi;
                    pose_candidates_call.request.observation_position.header.frame_id = m_world_frame_id;
                    pose_candidates_call.request.observation_position.header.stamp = ros::Time::now();
                    tf::pointTFToMsg(m_agent.camPoseForRobotPose(robot_pose).getOrigin(),
                                     pose_candidates_call.request.observation_position.point);
                    pose_candidates_call.request.lock_height = false;
                    while(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
                    {
                        logerror("get_observation_camera_poses service call failed (local samples)");
                        ros::Duration(5).sleep();
                    }
                    opc.addPoses(pose_candidates_call.response.camera_poses,
                                 pose_candidates_call.response.target_points,
                                 pose_candidates_call.response.cvms,
                                 pose_candidates_call.response.object_sets);
                }
                ROS_INFO("retrieving global pose candidates");
                {
                    uos_active_perception_msgs::GetObservationCameraPoses pose_candidates_call;
                    pose_candidates_call.request.sample_size = m_global_sample_size - persistent_samples.size();
                    pose_candidates_call.request.ray_skip = m_ray_skip;
                    pose_candidates_call.request.roi = goal.roi;
                    while(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
                    {
                        logerror("get_observation_camera_poses service call failed (global samples)");
                        ros::Duration(5).sleep();
                    }
                    opc.addPoses(pose_candidates_call.response.camera_poses,
                                 pose_candidates_call.response.target_points,
                                 pose_candidates_call.response.cvms,
                                 pose_candidates_call.response.object_sets);
                }
            }
            logtime("nbv_sampling_time", t0);
            logval("nbv_sample_count", opc.getPoses().size());

            if(!m_use_static_poses || persistent_samples.empty())
            {
                ROS_INFO("building travel time lookup tables");
                t0 = ros::WallTime::now();
                opc.prepareInitialTravelTimeLut(m_agent, robot_pose, cam_pose, m_world_frame_id);
                logtime("initial_tt_lut_time", t0);
                if(!m_log_dir.empty()) {
                    fname.str("");
                    fname.clear();
                    fname << m_log_dir << "/initial-tt-map-" << iteration_counter << ".tab";
                    opc.dumpInitialTravelTimeMap(fname.str());
                }
                size_t n_pruned = opc.pruneUnreachablePoses();
                logval("unreachable_poses_pruned", n_pruned);

                t0 = ros::WallTime::now();
                opc.prepareTravelTimeLut(m_agent, m_world_frame_id);
                logtime("mutual_tt_lut_time", t0);

                if(!m_ps_dir.empty())
                {
                    opc.serializeToFiles(m_ps_dir);
                    ROS_INFO_STREAM("Set " << opc.getPoses().size() << " persistent samples.");
                }
            }

            if(!opc.sanityCheck())
            {
                logerror("Transition time sanity check failed!");
            }

            // Find the number and total gain of discoverable cells using a greedy strategy
            double success_probability = 0.0;
            {
                double isuccess_probability = 1.0;
                detection_t observable_union = opc.observableUnion();
                for(detection_t::iterator it = observable_union.begin(); it != observable_union.end(); ++it)
                {
                    isuccess_probability *= (1.0 - rpcg(*it));
                }
                success_probability = 1.0 - isuccess_probability;
                logval("success_probability", success_probability);
            }

            // Termination criterion
            if(success_probability <= goal.min_p_succ)
            {
                loginfo("termination criterion: success_probability <= min_p_succ");
                m_observe_volumes_server.setSucceeded();
                return;
            }

            t0 = ros::WallTime::now();
            std::vector<size_t> plan;
            if(m_planning_mode == "search")
            {
                ROS_INFO("entering planning phase (search)");
                SearchPlanner<RegionalProbabilityCellGain> spl(rpcg, opc);
                double pdone_goal = 1.0 - (1.0 - success_probability) / (1.0 - goal.min_p_succ);
                double etime;
                bool finished = spl.makePlan(m_depth_limit,
                                             pdone_goal * m_relative_lookahead,
                                             m_max_rel_branch_cost,
                                             m_planning_timeout * 1000,
                                             plan, etime);
                if(!finished)
                {
                    logerror("planning timed out");
                }
            }
            else if(m_planning_mode == "greedy-reorder")
            {
                ROS_INFO("entering planning phase (greedy-reorder)");
                SearchPlanner<RegionalProbabilityCellGain> spl(rpcg, opc);
                double etime;
                spl.makeGreedy(plan, etime);
                bool finished = spl.optimalOrder(m_depth_limit,
                                                 success_probability * m_relative_lookahead,
                                                 m_max_rel_branch_cost,
                                                 m_planning_timeout * 1000,
                                                 plan, plan, etime);
                if(!finished)
                {
                    logerror("planning timed out");
                }
            }
            else
            {
                ROS_ERROR_STREAM("unknown planning mode: " << m_planning_mode);
            }
            logtime("planning_time", t0);

            // termination criterion
            if(plan.size() < 2)
            {
                loginfo("termination criterion: no plan");
                m_observe_volumes_server.setSucceeded();
                return;
            }

            // log and publish some plan details
            SearchPlan<RegionalProbabilityCellGain> sp(rpcg, opc);
            sp.pushSequence(plan, 1, plan.size());
            sp.deleteMarker(m_world_frame_id, m_marker_pub, "plan", marker_count);
            marker_count = sp.sendMarker(m_world_frame_id, m_marker_pub, "plan");
            if(!m_log_dir.empty()) {
                fname.str("");
                fname.clear();
                fname << m_log_dir << "/plan-timeplot-" << iteration_counter << ".tab";
                sp.writeTimeplot(fname.str());
            }

            size_t best_pose_idx = plan[1];
            logval("expected_move_time", opc.getInitialTravelTime(best_pose_idx));

            double iegain = 1.0;
            for(detection_t::iterator it = opc.getPoses()[best_pose_idx].cell_id_sets[0].begin();
                it != opc.getPoses()[best_pose_idx].cell_id_sets[0].end();
                ++it)
            {
                iegain *= (1.0 - rpcg(*it));
            }
            logval("expected_gain", 1.0 - iegain);

            // move the robot
            ros::Time st0 = ros::Time::now();
            if(m_agent.achieveCamPose(opc.getPoses()[best_pose_idx].pose,
                                        opc.getPoses()[best_pose_idx].view_distance))
            {
                // wait for acquisition
                ROS_INFO_STREAM("waiting " << m_agent.getAcquisitionTime() << " s for data acquisition...");
                ros::Duration(m_agent.getAcquisitionTime()).sleep();
            }
            else
            {
                logerror("failed to achieve target pose");
            }
            logval("actual_move_time", (ros::Time::now() - st0).toSec());

            // save expected view
            expected_view = opc.getPoses()[best_pose_idx].cell_id_sets[0];

            // save all poses that are part of the current plan.
            // index 0 is initial pose, index 1 is current target, so save poses beginning from index 2.
            if(m_keep_planned_poses && plan.size() > 2)
            {
                persistent_samples.resize(plan.size() - 2);
                for(size_t i = 2; i < plan.size(); ++i)
                {
                    tf::poseTFToMsg(opc.getPoses()[plan[i]].pose, persistent_samples[i-2]);
                }
            }

            // update opc
            opc.setCurrentPose(best_pose_idx);

            m_file_events.flush();
            m_file_vals.flush();
        }
    }
};

#endif // OBJECT_SEARCH_MANAGER_H
