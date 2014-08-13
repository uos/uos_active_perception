#include "object_search_manager.h"

#include <visualization_msgs/Marker.h>

ObjectSearchManager::ObjectSearchManager()
:
    m_node_handle("~"),
    m_node_handle_pub(),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/object_search_marker", 10000)),
    m_observe_volumes_server(m_node_handle_pub,
                             "observe_volumes",
                             boost::bind(&ObjectSearchManager::observeVolumesCb, this, _1),
                             false)
{
    m_observe_volumes_server.start();
}

void ObjectSearchManager::observeVolumesCb(race_object_search::ObserveVolumesGoalConstPtr const & goal)
{
    m_observe_volumes_server.setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_search_manager");
    ObjectSearchManager node;
    ROS_INFO("object_search_manager: Initialized!");
    ros::spin();
}
