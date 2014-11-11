#include "object_search_planner.h"

#include "observation_pose_collection.h"

#include <visualization_msgs/Marker.h>
#include <race_next_best_view/GetObservationCameraPoses.h>
#include <race_msgs/GetAnchoredObjects.h>

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
    m_agent()
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
    tf::StampedTransform robot_pose, cam_pose;
    m_tf_listener.lookupTransform(m_world_frame_id, m_agent.getRobotPoseFrameId(), ros::Time(0), robot_pose);
    m_tf_listener.lookupTransform(m_world_frame_id, m_agent.getRobotCameraFrameId(), ros::Time(0), cam_pose);

    ROS_INFO("retrieving pose candidates");
    ObservationPoseCollection opc;
    {
        race_next_best_view::GetObservationCameraPoses pose_candidates_call;
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
                     pose_candidates_call.response.cvms,
                     pose_candidates_call.response.object_sets);
    }

    ROS_INFO("building travel time lookup tables");
    opc.prepareInitialTravelTimeLut(m_agent, robot_pose, cam_pose, m_world_frame_id);
    opc.pruneUnreachablePoses();
    opc.prepareTravelTimeLut(m_agent, m_world_frame_id);

    ROS_INFO("Preparing greedy strategy");
    std::vector<double> time;
    std::vector<size_t> op_idx;
    std::vector<detection_t> detection;
    {
        double max_utility = 0.0;
        size_t max_utility_idx;
        for(size_t i = 0; i < opc.getPoses().size(); ++i) {
            double utility = opc.getPoses()[i].cell_id_sets[0].size() / opc.getInitialTravelTime(i);
            if(utility > max_utility) {
                max_utility = utility;
                max_utility_idx = i;
            }
        }
        op_idx.push_back(max_utility_idx);
        time.push_back(opc.getInitialTravelTime(max_utility_idx));
        detection.push_back(opc.getPoses()[max_utility_idx].cell_id_sets[0]);
    }

    while(time[time.size()-1] < HORIZON) {
        size_t last_idx = time.size() - 1;
        detection_t const & last_detection = detection[last_idx];
        double max_utility = 0.0;
        size_t max_utility_idx;
        double max_utility_duration = -1.0;
        detection_t max_utility_detection;
        for(size_t i = 0; i < opc.getPoses().size(); ++i) {
            if(i == op_idx[last_idx]) continue;
            double duration = opc.getTravelTime(op_idx[last_idx], i);
            detection_t detection_union;
            detection_union.insert(last_detection.begin(), last_detection.end());
            detection_union.insert(opc.getPoses()[i].cell_id_sets[0].begin(), opc.getPoses()[i].cell_id_sets[0].end());
            double utility = (detection_union.size() - last_detection.size()) / duration;
            if(utility > max_utility) {
                max_utility = utility;
                max_utility_idx = i;
                max_utility_duration = duration;
                max_utility_detection = detection_union;
            }
        }
        if(max_utility_duration < 1.0) {
            // Cannot find more useful observation poses
            break;
        }
        op_idx.push_back(max_utility_idx);
        time.push_back(time[last_idx] + max_utility_duration);
        detection.push_back(max_utility_detection);
    }

    ROS_INFO_STREAM("Greedy plan has " << time.size() << " steps and a duration of " << time[time.size()-1]);

    ROS_INFO("Sending marker");
    // Pose arrows
    for(size_t i = 0; i < time.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.2;
        if(i > 0) {
            marker.color.r = (double)detection[i-1].size() / (double)detection[i].size();
            marker.color.g = 1.0 - marker.color.r;
        } else {
            marker.color.b = 1.0;
        }
        marker.color.a = 0.5;
        tf::poseTFToMsg(opc.getPoses()[op_idx[i]].pose, marker.pose);
        marker.header.frame_id = m_world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.ns = "greedy_plan";
        m_marker_pub.publish(marker);
    }
    // Connecting lines
    for(size_t i = 0; i+1 < time.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.02;
        marker.scale.z = std::numeric_limits<float>::epsilon();
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        geometry_msgs::Point p;
        tf::pointTFToMsg(opc.getPoses()[op_idx[i]].pose.getOrigin(), p);
        marker.points.push_back(p);
        tf::pointTFToMsg(opc.getPoses()[op_idx[i+1]].pose.getOrigin(), p);
        marker.points.push_back(p);
        marker.header.frame_id = m_world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id = time.size() + i;
        marker.ns = "greedy_plan";
        m_marker_pub.publish(marker);
    }

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
