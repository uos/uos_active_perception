#include "object_search_manager.h"
#include "pr2_agent.h"
#include "floating_kinect_agent.h"
#include "turtlebot_agent.h"

int main(int argc, char** argv)
{
    std::string nname("object_search_manager");
    ros::init(argc, argv, nname);
    std::string robot;
    if(!ros::param::get("~robot", robot)) {
        ROS_ERROR_STREAM(nname << ": no robot specified!");
    }

    if(robot == "pr2")
    {
        ObjectSearchManager<Pr2Agent> node;
        ros::spin();
    }
    else if (robot == "floating_kinect")
    {
        ObjectSearchManager<FloatingKinectAgent> node;
        ros::spin();
    }
    else if (robot == "turtlebot")
    {
        ObjectSearchManager<TurtlebotAgent> node;
        ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM(nname << ": unsupported robot: " << robot);
    }
}
