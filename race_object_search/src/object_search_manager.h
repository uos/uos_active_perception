#ifndef OBJECT_SEARCH_MANAGER_H
#define OBJECT_SEARCH_MANAGER_H

#include "observation_pose_collection.h"
#include "search_plan.h"
#include "search_planner.h"

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
        m_observe_volumes_server(m_node_handle_pub,
                                 "observe_volumes",
                                 boost::bind(&ObjectSearchManager::observeVolumesCb, this, _1),
                                 false),
        m_agent(m_tf_listener, m_world_frame_id)
    {
        m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));
        m_node_handle.param("log_dir", m_log_dir, std::string(""));
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
    ros::Publisher m_marker_pub;
    actionlib::SimpleActionServer<race_object_search::ObserveVolumesAction> m_observe_volumes_server;
    TAgent m_agent;
    std::string m_world_frame_id;
    // sampling params
    int m_local_sample_size;
    int m_global_sample_size;
    double m_ray_skip;
    // planning params
    std::string m_planning_mode;
    int m_depth_limit;
    double m_relative_lookahead;
    double m_max_rel_branch_cost;
    double m_planning_timeout;

    // logging and evaluation data files
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

    struct MapRegion
    {
        uos_active_perception_msgs::CellId min, max;
        bool contains(uos_active_perception_msgs::CellId cell) const
        {
            return min.x <= cell.x && cell.x <= max.x &&
                   min.y <= cell.y && cell.y <= max.y &&
                   min.z <= cell.z && cell.z <= max.z;
        }
        size_t cellCount()
        {
            return (max.x - min.x + 1) * (max.y - min.y + 1) * (max.z - min.z + 1);
        }
    };

    struct MapRegionCollection
    {
        std::vector<MapRegion> regions;
        size_t findRegion(uos_active_perception_msgs::CellId cell) const
        {
            for(size_t i = 0; i < regions.size(); ++i)
            {
                if(regions[i].contains(cell)) return i;
            }
            return -1;
        }
    };

    struct RegionalProbabilityCellGain
    {
         const MapRegionCollection & map_region_collection;
         const std::vector<double> & cell_gains;

         RegionalProbabilityCellGain
         (
                 const MapRegionCollection & map_region_collection,
                 const std::vector<double> & cell_gains
         ):
             map_region_collection(map_region_collection),
             cell_gains(cell_gains)
         {}

         double operator ()(uint64_t const & cell) const
         {
             size_t idx = map_region_collection.findRegion(ObservationPoseCollection::cellIdIntToMsg(cell));
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

    std::vector<size_t> makePlanSimple(const ObservationPoseCollection & opc)
    {
        size_t best_pose_idx;
        double best_utility = 0;
        for(size_t i = 0; i < opc.getPoses().size(); i++)
        {
            double utility = opc.getPoses()[i].cell_id_sets[0].size() / opc.getInitialTravelTime(i);
            if(utility > best_utility)
            {
                best_pose_idx = i;
                best_utility = utility;
            }
        }
        if(best_utility > std::numeric_limits<double>::epsilon())
        {
            // The first value in a plan sequence is always the initial value and will be ignored, so insert 2 elements
            return std::vector<size_t>(2, best_pose_idx);
        }
        else
        {
            return std::vector<size_t>();
        }
    }

    // Callbacks
    void observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal_ptr)
    {
        ROS_INFO("Got a new goal!");
        size_t iteration_counter = 0;
        std::stringstream fname;
        ros::WallTime t0;
        size_t marker_count = 0;
        race_object_search::ObserveVolumesGoal const & goal = *goal_ptr.get();
        std::vector<geometry_msgs::Pose> persistent_samples;

        {
            std::ifstream iss("persistent_samples", std::ios::binary);
            if(iss.good())
            {

                iss.seekg (0, std::ios::end);
                std::streampos end = iss.tellg();
                iss.seekg (0, std::ios::beg);
                std::streampos begin = iss.tellg();
                size_t file_size = end-begin;
                boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
                iss.read((char*) ibuffer.get(), file_size);
                ros::serialization::IStream istream(ibuffer.get(), file_size);
                ros::serialization::deserialize(istream, persistent_samples);
                ROS_WARN_STREAM("Loaded " << persistent_samples.size() << " persistent samples");
            }
        }

        double cell_volume = 0.0;
        MapRegionCollection region_collection;
        region_collection.regions.resize(goal.roi.size());
        std::vector<double> unknown_roi_space(goal.roi.size(), 0.0);
        std::vector<double> roi_probability_density(goal.roi.size());
        std::vector<double> roi_cell_gain(goal.roi.size());
        for(size_t i = 0; i < goal.roi.size(); ++i)
        {
            roi_probability_density[i] = 1.0 - std::pow(1.0 - goal.p[i], 1.0 / (goal.roi[i].dimensions.x *
                                                                                goal.roi[i].dimensions.y *
                                                                                goal.roi[i].dimensions.z));

            uos_active_perception_msgs::GetBboxOccupancy get_bbox_occupancy;
            get_bbox_occupancy.request.bbox = goal.roi[i];
            while(!ros::service::call("/get_bbox_occupancy", get_bbox_occupancy))
            {
                logerror("get_bbox_occupancy service call failed, will try again");
                ros::WallDuration(5).sleep();
            }
            cell_volume = get_bbox_occupancy.response.cell_volume;
            region_collection.regions[i].min = get_bbox_occupancy.response.bbox_min;
            region_collection.regions[i].max = get_bbox_occupancy.response.bbox_max;
            roi_cell_gain[i] = 1.0 - std::pow(1.0 - goal.p[i], 1.0 / region_collection.regions[i].cellCount());
        }
        RegionalProbabilityCellGain rpcg(region_collection, roi_cell_gain);

        while(ros::ok())
        {
            if(m_observe_volumes_server.isPreemptRequested())
            {
                ROS_INFO("Preempted!");
                m_observe_volumes_server.setPreempted();
                return;
            }

            // evaluate actual information gain between iterations
            double gain = 0.0, probability_sum = 0.0;
            for(size_t i = 0; i < goal.roi.size(); ++i)
            {
                uos_active_perception_msgs::GetBboxOccupancy get_bbox_occupancy;
                get_bbox_occupancy.request.bbox = goal.roi[i];
                while(!ros::service::call("/get_bbox_occupancy", get_bbox_occupancy))
                {
                    logerror("get_bbox_occupancy service call failed, will try again");
                    ros::WallDuration(5).sleep();
                }
                probability_sum += (1.0 - probability_sum) * (1.0 - std::pow(1.0 - roi_probability_density[i],
                                                                             get_bbox_occupancy.response.unknown));
                gain += (1.0 - probability_sum) *
                        (1.0 - std::pow(1.0 - roi_probability_density[i],
                                              unknown_roi_space[i] - get_bbox_occupancy.response.unknown));
                unknown_roi_space[i] = get_bbox_occupancy.response.unknown;
            }
            if(iteration_counter > 0)
            {
                logval("gain", gain);
            }

            iteration_counter++;
            m_file_events << iteration_counter << std::endl;
            m_file_vals << iteration_counter << std::endl;

            logval("probability_sum", probability_sum);

            // get current robot pose
            const tf::Pose robot_pose = m_agent.getCurrentRobotPose();
            const tf::Pose cam_pose = m_agent.getCurrentCamPose();

            ObservationPoseCollection opc;
            size_t n_cells = 0;

            t0 = ros::WallTime::now();
            if(persistent_samples.empty())
            {
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
                    if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
                    {
                        logerror("get_observation_camera_poses service call failed (local samples)");
                        ros::Duration(5).sleep();
                        continue;
                    }
                    opc.addPoses(pose_candidates_call.response.camera_poses,
                                 pose_candidates_call.response.target_points,
                                 pose_candidates_call.response.cvms,
                                 pose_candidates_call.response.object_sets);
                    for(size_t i = 0; i < pose_candidates_call.response.roi_cell_counts.size(); ++i) {
                        n_cells += pose_candidates_call.response.roi_cell_counts[i];
                    }
                }
                ROS_INFO("retrieving global pose candidates");
                {
                    uos_active_perception_msgs::GetObservationCameraPoses pose_candidates_call;
                    pose_candidates_call.request.sample_size = m_global_sample_size;
                    pose_candidates_call.request.ray_skip = m_ray_skip;
                    pose_candidates_call.request.roi = goal.roi;
                    if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
                    {
                        logerror("get_observation_camera_poses service call failed (global samples)");
                        ros::Duration(5).sleep();
                        continue;
                    }
                    opc.addPoses(pose_candidates_call.response.camera_poses,
                                 pose_candidates_call.response.target_points,
                                 pose_candidates_call.response.cvms,
                                 pose_candidates_call.response.object_sets);
                }
                persistent_samples.resize(opc.getPoses().size());
                for(size_t i = 0; i < persistent_samples.size(); ++i)
                {
                    tf::poseTFToMsg(opc.getPoses()[i].pose, persistent_samples[i]);
                }

                // Write to disk
                std::ofstream oss("persistent_samples", std::ios::binary);
                size_t serial_size = ros::serialization::serializationLength(persistent_samples);
                boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
                ros::serialization::OStream ostream(obuffer.get(), serial_size);
                ros::serialization::serialize(ostream, persistent_samples);
                oss.write((char*) obuffer.get(), serial_size);
                oss.close();
                ROS_WARN_STREAM("Set " << persistent_samples.size() << " persistent samples");
            }
            else
            {
                ROS_INFO("reevaluating pose candidates");
                {
                    uos_active_perception_msgs::EvaluateObservationCameraPoses pose_candidates_call;
                    pose_candidates_call.request.camera_poses = persistent_samples;
                    pose_candidates_call.request.ray_skip = m_ray_skip;
                    pose_candidates_call.request.roi = goal.roi;
                    if(!ros::service::call("/evaluate_observation_camera_poses", pose_candidates_call))
                    {
                        logerror("evaluate_observation_camera_poses service call failed");
                        ros::Duration(5).sleep();
                        continue;
                    }
                    opc.addPoses(pose_candidates_call.response.camera_poses,
                                 pose_candidates_call.response.target_points,
                                 pose_candidates_call.response.cvms,
                                 pose_candidates_call.response.object_sets);
                    for(size_t i = 0; i < pose_candidates_call.response.roi_cell_counts.size(); ++i) {
                        n_cells += pose_candidates_call.response.roi_cell_counts[i];
                    }
                }
            }

            logtime("nbv_sampling_time", t0);
            logval("nbv_sample_count", opc.getPoses().size());

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
            if(!opc.sanityCheck()) {
                logerror("Transition time sanity check failed!");
            }

            // Find the number and total gain of discoverable cells using a greedy strategy
            double success_probability = 0.0;
            {
                std::vector<size_t> cell_counts(goal.roi.size(), 0);
                detection_t observable_union = opc.observableUnion();
                for(detection_t::iterator it = observable_union.begin(); it != observable_union.end(); ++it)
                {
                    success_probability += (1.0 - success_probability) * rpcg(*it);
                    size_t ridx = region_collection.findRegion(ObservationPoseCollection::cellIdIntToMsg(*it));
                    assert(ridx < cell_counts.size());
                    cell_counts[ridx]++;
                }

                // Termination criterion
                bool terminate = true;
                for(size_t i = 0; i < cell_counts.size(); ++i)
                {
                    //double observable_volume = cell_counts[i] * cell_volume;
                    if(cell_counts[i] > 0)//observable_volume >= goal.min_observable_volume)
                    {
                        terminate = false;
                        break;
                    }
                }
                if(terminate)
                {
                    loginfo("termination criterion: min_observable_volume");
                    m_observe_volumes_server.setSucceeded();
                    return;
                }
            }
            logval("success_probability", success_probability);

            ROS_INFO("entering planning phase");
            t0 = ros::WallTime::now();
            std::vector<size_t> plan;
            if(m_planning_mode == "simple")
            {
                plan = makePlanSimple(opc);
            }
            else if(m_planning_mode == "search")
            {
                SearchPlanner<RegionalProbabilityCellGain> spl(rpcg, opc);
                double etime;
                bool finished = spl.makePlan(m_depth_limit,
                                             success_probability * m_relative_lookahead,
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

            m_file_events.flush();
            m_file_vals.flush();
        }
    }
};

#endif // OBJECT_SEARCH_MANAGER_H
