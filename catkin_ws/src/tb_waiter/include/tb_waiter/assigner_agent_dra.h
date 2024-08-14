/*
 * assigner_agent_dra.h
 *
 *  Created on: Jul 30, 2014
 *      Author: chd
 */

#ifndef ASSIGNER_AGENT_DRA_H_
#define ASSIGNER_AGENT_DRA_H_

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <string>
#include <math.h>

#include <ecl/time.hpp>
#include <ecl/exceptions.hpp>
#include <termios.h> // for key_board input
#include <ecl/threads.hpp>

#include <ros/ros.h>
#include <tb_waiter/DynamicRoleAssigment.h>
#include <tb_waiter/DynamicRoleInfo.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Bool.h>

struct rbtUtilStruct
{
  std::string name;
  double utility;
  bool operator<(const rbtUtilStruct& rhs) const { return (utility < rhs.utility); }
  rbtUtilStruct():utility(0),name("-"){}
};

enum robotNames
{
  ROBOT_0,
  ROBOT_1,
  ROBOT_2,
  ROBOT_3,
  ROBOT_4
};

class AssignerAgentDra {
public:
  AssignerAgentDra();
  virtual ~AssignerAgentDra();

  void spin();

private:
  ros::NodeHandle nh_, ph_;
  ros::Publisher pub_announce_;
  ros::Subscriber sub_dra_status_r0_, sub_dra_status_r1_,sub_dra_status_r2_,sub_dra_status_r3_,sub_dra_status_r4_, sub_sim_coord_;

  int key_file_descriptor_;
  struct termios original_terminal_state_;
  ecl::Thread thread_;
  ros::Rate loop_rate_;
  std::string assigner_name_;

  std::vector<tb_waiter::DynamicRoleInfo> robot_status_;
  std::vector<rbtUtilStruct> robot_utilities_;
  //robot_status_
  //tb_waiter::DynamicRoleInfo robot_status_[5];

  char key_;
  bool request_memory_;
  bool simulation_;
  bool sim_monitor_request_;


protected:
  char getKey();
  void keyboardInputLoop();
  void mainProcess();
  bool waitUntilRequest();

  void robotStatus(int robot_num_id, tb_waiter::DynamicRoleInfo msg);
  int stringToInt(std::string s);
  rbtUtilStruct selectRobot(std::vector<rbtUtilStruct>  robot_utilities);
  //
  void robot0StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot1StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot2StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot3StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  void robot4StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg);
  //
  void simMonitorRequest(const std_msgs::Bool::ConstPtr& msg);

};

#endif /* ASSIGNER_AGENT_DRA_H_ */
