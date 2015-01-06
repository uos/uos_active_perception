/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Thorsten Gedicke
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

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
