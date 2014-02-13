#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asp_spatial_reasoning/GetBboxPercentUnseen.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"

#include <sstream>

bool get_bbox_percent_unseen(asp_spatial_reasoning::GetBboxPercentUnseen::Request &req,
                             asp_spatial_reasoning::GetBboxPercentUnseen::Response &res)
{
    res.percentUnseen = 0.5;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "asp_spatial_reasoner");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_bbox_percent_unseen", get_bbox_percent_unseen);

    ROS_INFO("asp_spatial_reasoner: Initialized!");

    ros::spin();
}
