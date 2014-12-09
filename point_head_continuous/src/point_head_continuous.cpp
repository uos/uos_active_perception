#include <ros/ros.h>
#include <ros/rate.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionServer<pr2_controllers_msgs::PointHeadAction> PointHeadServer;

class RobotHead
{
private:
  ros::NodeHandle handle;
  PointHeadClient point_head_client_;
  PointHeadServer point_head_server_;

public:

  RobotHead()
      : handle()
      , point_head_client_("/head_traj_controller/point_head_action", true)
      , point_head_server_(handle,
                           "/point_head_continuous/point_head_action",
                           boost::bind(&RobotHead::pointHeadCb, this, _1),
                           false)
  {
    //wait for head controller action server to come up
    while(!point_head_client_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
    point_head_server_.start();
  }

  void pointHeadCb(pr2_controllers_msgs::PointHeadGoalConstPtr const & goal_ptr)
  {
    ROS_INFO_STREAM("point_head_continuous goal received: stamp=" << goal_ptr.get()->target.header.stamp);
    pr2_controllers_msgs::PointHeadGoal goal(*goal_ptr.get());
    goal.target.header.stamp = ros::Time(); // always use most recent tf
    ros::Rate rate(20);
    while(ros::ok() && !point_head_server_.isPreemptRequested()) {
      rate.sleep();
      point_head_client_.sendGoal(goal);
    }
    point_head_server_.setAborted();
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "point_head_continuous");
  RobotHead head;
  ROS_INFO("point_head_continuous: Initialized!");
  ros::spin();
}
