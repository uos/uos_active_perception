#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/GetMultiplePlans.h>
#include <boost/random.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cstring>
#include <cstdlib>
#include <fstream>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client;
double timeout;

void mapCb(nav_msgs::OccupancyGrid const & map)
{
    std::string frame_id = map.header.frame_id;
    double max_x = map.info.width * map.info.resolution;
    double max_y = map.info.height * map.info.resolution;
    boost::mt19937 rng(std::time(0));
    boost::uniform_01<> r01;

    tf::TransformListener tf_listener;
    ros::Duration(2.0).sleep();

    // resolve TF prefix
    ros::NodeHandle nh_priv("~");
    std::string tf_prefix(tf::getPrefixParam(nh_priv));
    std::string base_frame_id = tf::resolve(tf_prefix, "base_footprint");

    std::ofstream f("random_driver_dump.tab", std::ofstream::trunc);
    f << "cost\ttime\tresult" << std::endl;

    ROS_INFO("map msg received, starting random driving");

    while(ros::ok())
    {
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = frame_id;
      target_pose.pose.position.x = max_x * r01(rng);
      target_pose.pose.position.y = max_y * r01(rng);
      target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.14 * r01(rng));

      tf::StampedTransform robot_pose;
      tf_listener.lookupTransform("map", base_frame_id, ros::Time(), robot_pose);
      geometry_msgs::PoseStamped start_pose;
      start_pose.header.frame_id = "map";
      tf::poseTFToMsg(robot_pose, start_pose.pose);

      move_base_msgs::GetMultiplePlans get_plans_call;
      get_plans_call.request.start.push_back(start_pose);
      get_plans_call.request.goal.push_back(target_pose);
      while(!ros::service::call("/move_base_planner_node/make_multiple_plans", get_plans_call))
      {
          ROS_WARN("make_multiple_plans call failed, will retry");
          ros::WallDuration(5.0).sleep();
      }
      if(get_plans_call.response.plans[0].poses.empty())
      {
          ROS_INFO_STREAM("Skipping unreachable goal, cost: " << get_plans_call.response.plan_costs[0]);
          continue;
      }

      ROS_INFO_STREAM("driving to " << target_pose.pose.position);

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = target_pose;
      move_base_client->sendGoal(goal);
      ros::Time t0 = ros::Time::now();
      move_base_client->waitForResult(ros::Duration(timeout));
      if(move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          f << get_plans_call.response.plan_costs[0] << "\t" << ros::Time::now() - t0 << "\tsuccess" << std::endl;
          ROS_INFO("goal reached with success");
      }
      else
      {
          f << get_plans_call.response.plan_costs[0] << "\t" << ros::Time::now() - t0 << "\tfail" << std::endl;
          ROS_INFO("goal approach failed");
          move_base_client->cancelAllGoals();
      }

      f.flush();
    }

    f.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_driver");
    move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    ros::param::param("~timeout", timeout, 60.0);
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("/map", 1, &mapCb);
    ros::spin();
    delete move_base_client;
}

