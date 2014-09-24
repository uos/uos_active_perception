#include "object_search_manager.h"

#include <visualization_msgs/Marker.h>
#include <race_next_best_view/GetObservationCameraPoses.h>

ObjectSearchManager::ObjectSearchManager()
:
    m_node_handle("~"),
    m_node_handle_pub(),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/object_search_marker", 10000)),
    m_observe_volumes_server(m_node_handle_pub,
                             "observe_volumes",
                             boost::bind(&ObjectSearchManager::observeVolumesCb, this, _1),
                             false),
    m_agent()
{
    m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));

    m_observe_volumes_server.start();
}

void ObjectSearchManager::observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal_ptr)
{
    ROS_INFO("Got a new goal!");
    race_object_search::ObserveVolumesGoal const & goal = *goal_ptr.get();
    while(ros::ok())
    {
        if(m_observe_volumes_server.isPreemptRequested())
        {
            ROS_INFO("Preempted!");
            m_observe_volumes_server.setPreempted();
            return;
        }

        // get current robot pose
        tf::StampedTransform robot_pose, cam_pose;
        m_tf_listener.lookupTransform(m_world_frame_id, m_agent.getRobotPoseFrameId(), ros::Time(0), robot_pose);
        m_tf_listener.lookupTransform(m_world_frame_id, m_agent.getRobotCameraFrameId(), ros::Time(0), cam_pose);

        std::vector<geometry_msgs::Pose> pose_candidates;
        std::vector<double> candidate_information_gain;

        ROS_INFO("retrieving local pose candidates");
        {
            race_next_best_view::GetObservationCameraPoses pose_candidates_call;
            pose_candidates_call.request.sample_size = 100;
            pose_candidates_call.request.ray_skip = 0.75;
            pose_candidates_call.request.roi = goal.roi;
            pose_candidates_call.request.observation_position.header.frame_id = m_world_frame_id;
            pose_candidates_call.request.observation_position.header.stamp = ros::Time::now();
            tf::pointTFToMsg(m_agent.cam_pose_for_robot_pose(robot_pose).getOrigin(),
                             pose_candidates_call.request.observation_position.point);
            pose_candidates_call.request.lock_height = false;
            if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
            {
                ROS_ERROR("service call failed");
                ros::Duration(5).sleep();
                continue;
            }
            pose_candidates.insert(pose_candidates.end(),
                                   pose_candidates_call.response.camera_poses.begin(),
                                   pose_candidates_call.response.camera_poses.end());
            candidate_information_gain.insert(candidate_information_gain.end(),
                                              pose_candidates_call.response.information_gain.begin(),
                                              pose_candidates_call.response.information_gain.end());
        }

        ROS_INFO("retrieving global pose candidates");
        {
            race_next_best_view::GetObservationCameraPoses pose_candidates_call;
            pose_candidates_call.request.sample_size = 200;
            pose_candidates_call.request.ray_skip = 0.75;
            pose_candidates_call.request.roi = goal.roi;
            if(!ros::service::call("/get_observation_camera_poses", pose_candidates_call))
            {
                ROS_ERROR("service call failed");
                ros::Duration(5).sleep();
                continue;
            }
            pose_candidates.insert(pose_candidates.end(),
                                   pose_candidates_call.response.camera_poses.begin(),
                                   pose_candidates_call.response.camera_poses.end());
            candidate_information_gain.insert(candidate_information_gain.end(),
                                              pose_candidates_call.response.information_gain.begin(),
                                              pose_candidates_call.response.information_gain.end());
        }

        ROS_INFO("evaluating pose candidate utility values");
        std::vector<double> candidate_utilities(pose_candidates.size());
        tf::Pose best_cam_pose; best_cam_pose.setIdentity();
        double best_utility = 0;
        for(unsigned int i = 0; i < candidate_utilities.size(); i++)
        {
            tf::Pose pose_candidate_cam_tf;
            tf::poseMsgToTF(pose_candidates[i], pose_candidate_cam_tf);

            candidate_utilities[i] = candidate_information_gain[i] /
                                     m_agent.estimate_move_time(robot_pose,
                                                                cam_pose,
                                                                pose_candidate_cam_tf,
                                                                m_world_frame_id);

            if(candidate_utilities[i] > best_utility)
            {
                best_cam_pose = pose_candidate_cam_tf;
                best_utility = candidate_utilities[i];
            }
        }

        if(best_utility == 0)
        {
            m_observe_volumes_server.setSucceeded();
            return;
        }

        // do it
        if(m_agent.achieve_cam_pose(robot_pose, cam_pose, best_cam_pose, m_world_frame_id))
        {
            // wait for acquisition
            ros::Duration(m_agent.get_acquisition_time()).sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_search_manager");
    ObjectSearchManager node;
    ROS_INFO("object_search_manager: Initialized!");
    ros::spin();
}
