#ifndef OBJECT_SEARCH_MANAGER_H
#define OBJECT_SEARCH_MANAGER_H

#include "observation_pose_collection.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <race_object_search/ObserveVolumesAction.h>
#include <visualization_msgs/Marker.h>
#include <uos_active_perception_msgs/GetObservationCameraPoses.h>

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
        m_node_handle.param("local_sample_size", m_local_sample_size, 100);
        m_node_handle.param("global_sample_size", m_global_sample_size, 200);
        m_node_handle.param("ray_skip", m_ray_skip, 0.75);

        // open logging and evaluation data files
        if(!m_log_dir.empty()) {
            std::stringstream fname;
            fname << m_log_dir << "/events.log";
            m_file_events.open(fname.str().c_str(), std::ofstream::trunc);
            fname.str("");
            fname.clear();
            fname << m_log_dir << "/vals.log";
            m_file_vals.open(fname.str().c_str(), std::ofstream::trunc);
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
    int m_local_sample_size;
    int m_global_sample_size;
    double m_ray_skip;

    // logging and evaluation data files
    std::string m_log_dir;
    std::ofstream m_file_events;
    std::ofstream m_file_vals;

    void logerror(const std::string & str)
    {
        ROS_ERROR_STREAM(str.c_str());
        if(m_file_events.is_open()) {
            m_file_events << ros::WallTime::now() << " ERROR: " << str << std::endl;
        }
    }

    void loginfo(const std::string & str)
    {
        ROS_INFO_STREAM(str.c_str());
        if(m_file_events.is_open()) {
            m_file_events << ros::WallTime::now() << " -----: " << str << std::endl;
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

    // Callbacks
    void observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal_ptr)
    {
        ROS_INFO("Got a new goal!");
        size_t iteration_counter = 0;
        std::stringstream fname;
        ros::WallTime t0;
        race_object_search::ObserveVolumesGoal const & goal = *goal_ptr.get();
        while(ros::ok())
        {
            if(m_observe_volumes_server.isPreemptRequested())
            {
                ROS_INFO("Preempted!");
                m_observe_volumes_server.setPreempted();
                return;
            }

            iteration_counter++;
            m_file_events << iteration_counter << std::endl;
            m_file_vals << iteration_counter << std::endl;

            // get current robot pose
            const tf::Pose robot_pose = m_agent.getCurrentRobotPose();
            const tf::Pose cam_pose = m_agent.getCurrentCamPose();

            ObservationPoseCollection opc;

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

            ROS_INFO("entering planning phase");
            t0 = ros::WallTime::now();
            std::vector<double> candidate_utilities(opc.getPoses().size());
            size_t best_pose_idx;
            double best_utility = 0;
            for(size_t i = 0; i < candidate_utilities.size(); i++)
            {
                candidate_utilities[i] = opc.getPoses()[i].cell_id_sets[0].size() /
                                         opc.getInitialTravelTime(i);

                if(candidate_utilities[i] > best_utility)
                {
                    best_pose_idx = i;
                    best_utility = candidate_utilities[i];
                }
            }
            logtime("planning_time", t0);

            // termination criterion
            if(best_utility == 0)
            {
                loginfo("termination criterion reached");
                m_observe_volumes_server.setSucceeded();
                return;
            }

            logval("expected_move_time", opc.getInitialTravelTime(best_pose_idx));

            // move the robot
            ros::Time st0 = ros::Time::now();
            if(m_agent.achieveCamPose(opc.getPoses()[best_pose_idx].pose,
                                        opc.getPoses()[best_pose_idx].view_distance))
            {
                // wait for acquisition
                ros::Duration(m_agent.getAcquisitionTime()).sleep();
            }
            else
            {
                logerror("failed to achieve target pose");
            }
            logval("actual_move_time", (ros::Time::now() - st0).toSec());
        }
    }
};

#endif // OBJECT_SEARCH_MANAGER_H
