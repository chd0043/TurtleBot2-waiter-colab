/*
 * kobuki_command.h
 *
 *  Created on: Jul 7, 2014
 *    Author: chd
 */

#ifndef KOBUKICOMMANDS_H_
#define KOBUKICOMMANDS_H_

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <string>
#include <math.h>
#include <algorithm>
#include <exception>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingGoal.h>
#include <tb_waiter/TurtlebotMoveAction.h>

#include <std_srvs/Empty.h>

#include <tb_waiter/robPosInfo.h>
#include <tb_waiter/robPlanDist.h>
#include <tb_waiter/robResetPos.h>
#include <tb_waiter/cvMarkRec.h>

#include <ros/console.h>

typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;
typedef actionlib::SimpleClientGoalState SimpleClientGoalState;

class RobPosition
{
public:
  RobPosition():
   x(0),y(0), theta(0), dist(0)
  {}

  RobPosition(double x_i, double y_i, double theta_i=0, double dist_i=0):
   x(x_i),y(y_i), theta(theta_i), dist(dist_i)
  {}

  double x;
  double y;
  double theta;
  double dist;

  //
  bool operator==(const RobPosition& rhs) const
  {
    return ( x == rhs.x && y == rhs.y );
  }

  bool operator!=(const RobPosition& rhs) const
  {
    return ( x != rhs.x || y != rhs.y );
  }

  RobPosition operator+(RobPosition p)
  {
     return ( RobPosition(x+p.x, y+p.y) );
  }

  RobPosition operator-(RobPosition p)
  {
     return ( RobPosition(x-p.x, y-p.y) );
  }
};

struct cvLandMark {
  std::string figure;
  std::string color;
};


class KobukiCommand
{
public:
  KobukiCommand(std::string frameID="/map");
  virtual ~KobukiCommand();

	//
  void cancelGoal();
  void navigateToGoals(std::vector<RobPosition> goals);
  bool navigateToGoal(RobPosition goal);
  void sendMoveBaseGoal(RobPosition goal);
  void waitForMoveBaseResult(const ros::Duration& timeout);
  bool waitForMoveBaseResult();
  bool getMoveBaseDoneStatus();
  //
  bool clearCostmaps();
  bool cleanupAndError();
  bool enableRecovery();
  bool disableRecovery();
  bool softRecovery();
  bool hardRecovery();
  //
  tf::StampedTransform getTf(const std::string& frame_1, const std::string& frame_2);
  tf::StampedTransform getOdomTf();
  tf::StampedTransform getRobotTf();

  // ############################################################## //

  bool turtlebotMove(double turn_distance, double forward_distance);
  void turnLeft();
  void turnRigth();
  void moveForward(double forward_distance);
  void turnAngle(double turn_distance);
  void moveBaseXY(RobPosition goal_pose);

  // ############################################################## //

  RobPosition getPosActual();
  RobPosition readRobPos(std::string infoSource);
  double estimateCost(RobPosition goal);
  double getMeasuredDistance();
  void resetOdom();
  void setInitialPose(int num);
  void kobukiCommand(std::string command);

  // ############################################################## //

  cvLandMark getLandMark(std::string command="");
  int readRefLandMark();

  // ############################################################## //

  bool rAutodocking();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle ph_;
  std::string frameID_;
  std::string robot_name_;
  bool simulation_;
  bool enable_mark_;
  bool move_base_done_status_;
  bool recovery_behavior_;

  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  tf::TransformListener tf_listener_;

  //
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mbAC_;
  actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> adAC_;
  actionlib::SimpleActionClient<tb_waiter::TurtlebotMoveAction> tbAC_;

  // ############################################################## //

  MoveBaseGoal createMoveBaseGoal(double x, double y, std::string frame_id="/map");
  double calcRotationAngle(double x, double y);

  // ############################################################## //
  template <typename T>
  bool cancelAllGoals(actionlib::SimpleActionClient<T> & action_client, double timeout = 2.0);
  template <typename T>
  bool waitForServer(actionlib::SimpleActionClient<T> & action_client, double timeout = 2.0);
  template <typename T>
  const char* acName(actionlib::SimpleActionClient<T> & action_client);
  template <typename S,typename T>
  double heading(const S &a, const T &b);

  // ############################################################## //
  double wrapAngle(double a);
  //
  void moveBaseDoneCb(const actionlib::SimpleClientGoalState& status,
                      const move_base_msgs::MoveBaseResultConstPtr & result);
  void moveBaseActiveCb();
  void moveBaseFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr & feedback);

  void dockDoneCb(const actionlib::SimpleClientGoalState& status, const kobuki_msgs::AutoDockingResultConstPtr & result);
  void dockActiveCb();
  void dockFeedbackCb(const kobuki_msgs::AutoDockingFeedbackConstPtr & feedback);
  void dockDriveClient();
  void dockingSequence();

  // ############################################################## //
};

#endif /* ROBOTCOMMANDS_H_ */
