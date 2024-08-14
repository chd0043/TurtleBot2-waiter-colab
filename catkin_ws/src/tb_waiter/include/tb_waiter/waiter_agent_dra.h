/*
 * waiter_agent_dra.h
 *
 *  Created on: Jul 29, 2014
 *      Author: chd
 */

#ifndef WAITER_AGENT_DRA_H_
#define WAITER_AGENT_DRA_H_


#include <iostream>
#include <signal.h>
#include <termios.h>
#include <string>
#include <math.h>
#include <algorithm>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tb_waiter/DynamicRoleAssigment.h>
#include <tb_waiter/DynamicRoleInfo.h>
#include <tb_waiter/kobuki_command.h>
#include <tb_waiter/datalogger.hpp>

const double MAX_U = 99.9;

struct draRequestStruct
{
  int table_id_num;
  std::string name;
  std::string type;
  std::string status;
  std::string contract;
  double utility;
  geometry_msgs::PoseWithCovariance pose;
  bool operator<(const draRequestStruct& rhs) const { return (utility < rhs.utility); }
  draRequestStruct():utility(0),table_id_num(0){}
};

enum FsmState
{
  ST0_IDLE,
  ST1_DOCK,
  ST2_WAITING,
  ST3_DELIVER,
  ST4_HOME
};

enum table_id
{
  TABLE0,
  TABLE1,
  TABLE2,
  TABLE3,
  TABLE4
};

enum robotNames
{
   ROBOT_0,
   ROBOT_1,
   ROBOT_2,
   ROBOT_3,
   ROBOT_4
};

namespace dra
{
  struct candidateStruct
  {
    std::string emitter;
    RobPosition position;
    double distance;
    bool operator<(const candidateStruct& rhs) const { return (distance < rhs.distance); }
    candidateStruct():emitter(""),distance(0){}
  };
}

//typedef FsmStates::FsmState FsmState;
typedef dra::candidateStruct candidateStruct;


class WaiterAgentDra
{
public:
  WaiterAgentDra();
  virtual ~WaiterAgentDra();

  void spin();

private:
  int dra_state_;
  double current_utility_;
  std::string current_state_;
  RobPosition current_goal_;

  ros::Subscriber sub_announce_t0_, sub_announce_t1_, sub_announce_t2_, sub_announce_t3_, sub_announce_t4_;
  ros::Subscriber sub_dra_status_r0_, sub_dra_status_r1_,sub_dra_status_r2_,sub_dra_status_r3_,sub_dra_status_r4_;
  ros::Publisher pub_announce_, pub_dra_status_;
  ros::NodeHandle nh_, ph_;
  ros::Rate loop_rate_;
  ros::Timer periodic_func_;
  //
  std::string robot_name_;
  std::string assigned_table_;
  ros::Time begin_time_;

  KobukiCommand kobuki_base_;
  RobPosition initial_pose_;

  DataLogger csv_logger_;
  dataStruct dataline_;

  bool bRobotFree_;
  bool simulation_;

  std::vector<tb_waiter::DynamicRoleInfo> dra_status_list_;
  std::vector<draRequestStruct> tables_req_service_;
  tb_waiter::DynamicRoleInfo dra_my_status_;
  std::vector<double> utility_penalty_;

  void finiteStateMachine();
  RobPosition selectTable(std::vector<draRequestStruct> tables_req_service);
  bool waitForRequest();
  bool waitTime(double time_preset);

  //
  void assignerAnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  void table0AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  void table1AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  void table2AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  void table3AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  void table4AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  //
  void tableAnnounceAddToList(int table_id_num, const tb_waiter::DynamicRoleAssigment::ConstPtr& msg);
  //
  void robot0StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot1StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot2StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot3StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot4StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  //
  void robotStatus(int robot_num_id, tb_waiter::DynamicRoleInfo msg);
  //
  void periodicCallback(const ros::TimerEvent& event);
  //
  double estimateIdleStateUtility();
  double estimateDockerStateUtility();
  double estimateWaiterStateUtility();
  double estimateHomeStateUtility();
  double estimateDeliverStateUtility(int table_id_num);

  double estimateDistance(geometry_msgs::PoseWithCovariance position);
  double estimateDistance(RobPosition position);
  double estimateDistance(RobPosition position1, RobPosition position2);

  // State machine steps
  void fsmSt0Idle();
  void fsmSt1Dock();
  void fsmSt2Waiting();
  void fsmSt3Deliver();
  void fsmSt4Home();
  //
  int lookupTransitions(int cur_state);
  double waitForTransition();

protected:

};

#endif /* WAITER_AGENT_DRA_H_ */
