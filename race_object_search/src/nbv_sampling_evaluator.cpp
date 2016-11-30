#include "observation_pose_collection.h"
#include "geometry.h"
#include "floating_kinect_agent.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <race_object_search/ObserveVolumesAction.h>
#include <uos_active_perception_msgs/GetObservationCameraPoses.h>

#include <fstream>

class NbvSamplingEvaluator
{
public:
    NbvSamplingEvaluator()
    :
        m_node_handle("~"),
        m_node_handle_pub(),
        m_observe_volumes_server(m_node_handle_pub,
                                 "observe_volumes",
                                 boost::bind(&NbvSamplingEvaluator::observeVolumesCb, this, _1),
                                 false),
        m_tfl(),
        m_agent(m_tfl, m_world_frame_id)
    {
        m_node_handle.param("world_frame_id", m_world_frame_id, std::string("odom_combined"));
        // sampling params
        m_node_handle.param("global_sample_size", m_global_sample_size, 1000);
        m_node_handle.param("ray_skip", m_ray_skip, 0.0);
        m_node_handle.param("n", n, 1);
        m_node_handle.param("filter_unreachable", m_filter_unreachable, false);

        m_observe_volumes_server.start();
        std::cout << "READY" << std::endl;
    }

private:
    ros::NodeHandle m_node_handle, m_node_handle_pub;
    actionlib::SimpleActionServer<race_object_search::ObserveVolumesAction> m_observe_volumes_server;
    tf::TransformListener m_tfl;
    FloatingKinectAgent m_agent;
    std::string m_world_frame_id;
    // sampling params
    int n;
    int m_global_sample_size;
    double m_ray_skip;
    bool m_filter_unreachable;

    void observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal_ptr)
    {
        using namespace geometry;

        std::cout << "new" << std::endl;
        race_object_search::ObserveVolumesGoal const & goal = *goal_ptr.get();

        for(int i = 0; i < n; ++i)
        {
            // get samples
            uos_active_perception_msgs::GetObservationCameraPoses pose_candidates_call;
            pose_candidates_call.request.sample_size = m_global_sample_size;
            pose_candidates_call.request.ray_skip = m_ray_skip;
            pose_candidates_call.request.roi = goal.roi;
            pose_candidates_call.request.keep_blind_poses = true;
            ros::WallTime t0 = ros::WallTime::now();
            if(!ros::service::call("get_observation_camera_poses", pose_candidates_call))
            {
                std::cout << "ERROR" << std::endl;
                m_observe_volumes_server.setAborted();
                return;
            }
            std::cout << "response," << (ros::WallTime::now() - t0).toSec() << ",";
            ObservationPoseCollection opc;
            opc.addPoses(pose_candidates_call.response.camera_poses,
                         pose_candidates_call.response.target_points,
                         pose_candidates_call.response.cvms,
                         pose_candidates_call.response.object_sets);
            std::cout << opc.getPoses().size() << std::endl;

            // calc travel times
            if(m_filter_unreachable)
            {
                opc.prepareInitialTravelTimeLut(m_agent,
                                                m_agent.getCurrentRobotPose(),
                                                m_agent.getCurrentCamPose(),
                                                m_world_frame_id);
            }

            // print coverage curve
            std::cout << "cells,";
            detection_t all;
            for(size_t k = 0; k < opc.getPoses().size(); ++k)
            {
                std::cout << all.size() << ",";
                if(!m_filter_unreachable || opc.getInitialTravelTime(k) < std::numeric_limits<double>::infinity())
                {
                    const detection_t & ins = opc.getPoses()[k].cell_id_sets[0];
                    all.insert(ins.begin(), ins.end());
                }
            }
            std::cout << all.size() << std::endl;
        }

        m_observe_volumes_server.setSucceeded();
    }
};

int main(int argc, char** argv)
{
    std::string nname("nbv_sampling_evaluator");
    ros::init(argc, argv, nname);
    NbvSamplingEvaluator node;
    ros::spin();
}
