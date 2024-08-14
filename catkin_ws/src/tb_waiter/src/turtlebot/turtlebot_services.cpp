#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <ros/param.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/foreach.hpp>
#include <string>
#include <math.h>
#include <tb_waiter/robPosInfo.h>
#include <tb_waiter/robPlanDist.h>
#include <tb_waiter/robResetPos.h>
#include <tb_waiter/kobuki_base.h>

const std::string SRV_MAKE_PLAN = "move_base/make_plan";

using namespace kobuki;


class TurtlebotServices : public kobukiBase
{
private:
  ros::ServiceClient make_plan_client_;
  std::string global_frame_id_, tf_prefix_;

  ros::NodeHandle nh_, ph_;
  ros::Rate loop_rate_;
  ros::ServiceServer get_pos_service_;
  ros::ServiceServer make_plan_service_;
  ros::ServiceServer reset_robot_pose_service_;

public:
  TurtlebotServices():
    loop_rate_(10),
    ph_("~")
  {
    tf_prefix_ = tf::getPrefixParam(nh_);

    ph_.param("global_frame_id", global_frame_id_, std::string("map"));
    //global_frame_id_ = tf::resolve(tf_prefix_, global_frame_id_);

    while (!ros::service::waitForService(SRV_MAKE_PLAN, ros::Duration(3.0))) {
      ROS_INFO("Waiting for service '~make_plan' to become available");
    }

    make_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>(SRV_MAKE_PLAN, true);
    if (!make_plan_client_) {
      ROS_FATAL("Could not initialize get plan service from %s", make_plan_client_.getService().c_str());
      return;
    }

    get_pos_service_          = ph_.advertiseService("robPosInfo",  &TurtlebotServices::sendPosInfo,     this);
    make_plan_service_        = ph_.advertiseService("robPlanDist", &TurtlebotServices::sendDistInfo,    this); // resetRobotPosInfo
    reset_robot_pose_service_ = ph_.advertiseService("robResetPos", &TurtlebotServices::resetRobotPosInfo, this);
  }

  void spin(){
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate_.sleep();
    }
  }

protected:
  //----------------------------------------------------------------------
  void fillPathRequest(const std::string frame_id,const robPosition goal_position ,nav_msgs::GetPlan::Request &request)
  {
    double g_GoalTolerance = 0.5;

    tf::Quaternion quaternionStart, quaternionGoal;
    quaternionStart = tf::createQuaternionFromRPY(0, 0, amcl_pose_.theta);
    quaternionGoal = tf::createQuaternionFromRPY(0, 0, goal_position.theta);

    request.start.header.frame_id = frame_id;
    request.start.pose.position.x = amcl_pose_.x;
    request.start.pose.position.y = amcl_pose_.y;
    request.start.pose.orientation.w = 1.0;

    request.goal.header.frame_id = frame_id;
    request.goal.pose.position.x = goal_position.x;
    request.goal.pose.position.y = goal_position.y;
    request.goal.pose.orientation.w = 1.0;

    request.tolerance = g_GoalTolerance;
  }

  double callPlanningService(ros::ServiceClient &make_plan_client, const robPosition &goal_position)
  {
    nav_msgs::GetPlan srv;
    fillPathRequest(global_frame_id_, goal_position, srv.request);

    double path_length = 0;
    robPosition last_pose = amcl_pose_;

    // Perform the actual path planner call
    if (make_plan_client.call(srv)) {
      ROS_INFO("x = %f, y = %f", amcl_pose_.x, amcl_pose_.y);
      if (!srv.response.plan.poses.empty()) {
        BOOST_FOREACH(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
          path_length += hypot( p.pose.position.x - last_pose.x,
                                p.pose.position.y - last_pose.y );
          last_pose.x = p.pose.position.x;
          last_pose.y = p.pose.position.y;
          ROS_DEBUG("x = %f, y = %f, dist = %f", p.pose.position.x, p.pose.position.y, path_length);
        }
      }
      else {
        ROS_WARN("Got empty plan");
        return (0);
      }
    }
    else
    {
      ROS_WARN("Failed to call service %s - is the robot moving? Calculates instead Manhattan distance.", make_plan_client_.getService().c_str());
      // Euclidian distance
      //path_length = hypot(goal_position.x - amcl_pose_.x,
      //                    goal_position.y - amcl_pose_.y );
      // Manhattan distance
      path_length = fabs(goal_position.x - amcl_pose_.x) +
                    fabs(goal_position.y - amcl_pose_.y);
      ROS_INFO("x = %f, y = %f, dist = %f", goal_position.x, goal_position.y, path_length);
    }
    return (path_length);
  }

  //----------------------------------------------------------------------

  bool sendPosInfo(tb_waiter::robPosInfo::Request  &req,
                   tb_waiter::robPosInfo::Response &res)
  {
    if (req.topic == "amcl"){
      res.x = amcl_pose_.x;
      res.y = amcl_pose_.y ;
      res.theta = amcl_pose_.theta;
      res.dist = amcl_pose_.dist;
    }
    else
    {
      res.x = odom_pose_.x;
      res.y = odom_pose_.y ;
      res.theta = odom_pose_.theta;
      res.dist = odom_pose_.dist;
    }
    return (true);
  }

  bool sendDistInfo(tb_waiter::robPlanDist::Request  &req,
                    tb_waiter::robPlanDist::Response  &res)
  {
    nav_msgs::GetPlan srv;
    robPosition goal_position;
    goal_position.x = req.x;
    goal_position.y = req.y;
    goal_position.theta = 0;

    if (!make_plan_client_)
    {
      ROS_FATAL("Persistent service connection to %s failed", make_plan_client_.getService().c_str());
      return (false);
    }

    res.dist = callPlanningService(make_plan_client_, goal_position);
    return (true);
  }

  bool resetRobotPosInfo(tb_waiter::robResetPos::Request  &req,
                         tb_waiter::robResetPos::Response  &res)
  {
    if (req.command == "reset_Odom")
      resetOdom();
    if (req.command == "initialpose1")
      setInitialPose1();
    if (req.command == "initialpose2")
      setInitialPose2();
    if (req.command == "cancelGoal")
      cancelMoveBaseGoal();
    res.ok = 0;

    return (true);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ROS_server");

  TurtlebotServices server;

  ROS_INFO("Turtlebot Services");
  //
  server.spin();
  //ros::spin();

  return (0);
}
