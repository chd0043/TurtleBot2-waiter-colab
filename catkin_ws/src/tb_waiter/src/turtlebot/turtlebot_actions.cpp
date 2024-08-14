/*
 * turtlebotActions.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: chd
 */


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <tb_waiter/TurtlebotMoveAction.h>
#include <tb_waiter/kobuki_base.h>
#include <cmath>

using namespace kobuki;

class MoveActionServer : public kobukiBase
{
private:

  ros::NodeHandle nh_, ph_;
  ros::Rate loop_rate_;
  actionlib::SimpleActionServer<tb_waiter::TurtlebotMoveAction> as_;
  std::string action_name_;

  tb_waiter::TurtlebotMoveFeedback     feedback_;
  tb_waiter::TurtlebotMoveResult       result_;
  tb_waiter::TurtlebotMoveGoalConstPtr goal_;

public:
  MoveActionServer(const std::string name) :
      ph_("~"),
	  as_(nh_, name, false),
	  action_name_(name),
	  loop_rate_(10)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&MoveActionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveActionServer::preemptCB, this));

    as_.start();
  }

  void spin()
  {
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate_.sleep();
	}
  }

protected:
  void goalCB()
  {
    // accept the new goal
    feedback_.forward_distance = 0.0;
    feedback_.turn_distance = 0.0;

    result_.forward_distance = 0.0;
    result_.turn_distance = 0.0;

    goal_ = as_.acceptNewGoal();

    if (!turnController(goal_->turn_distance))
    {
      as_.setAborted(result_);
      return;
    }
    ros::Duration(0.5).sleep();

    if (fwdController(goal_->forward_distance))
      as_.setSucceeded(result_);
    else
      as_.setAborted(result_);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_move");

  MoveActionServer server("turtlebot_move");

  ROS_INFO("turtlebot_move");
  //server.spin();
  ros::spin();

  return (0);
}
