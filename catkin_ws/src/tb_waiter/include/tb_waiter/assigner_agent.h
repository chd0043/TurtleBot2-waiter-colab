/*
 * AssignerAgent.h
 *
 *  Created on: Jun 30, 2014
 *    Author: chd
 */

#ifndef ASSIGNERAGENT_H_
#define ASSIGNERAGENT_H_

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
#include <std_msgs/Bool.h>
#include <tb_waiter/ContractNetMsg.h>

struct robPosition {
  double x;
  double y;
  double theta;
  double dist;
  robPosition():x(0),y(0),
      theta(0), dist(0) {}
};

struct candidateStruct{
  std::string emitter;
  double offer;
  bool operator<(const candidateStruct& a) const { return (offer < a.offer); }
  //inline bool operator>(const candidateStruct& a)  { return (offer > a.offer); }
  candidateStruct():emitter(""),offer(0){}
};

class AssignerAgent {

public:
  AssignerAgent();
  virtual ~AssignerAgent();

  void spin();

private:
  ros::Subscriber sub_refusal_, sub_proposal_, sub_received_, sub_award_resp_, sub_sim_coord_;
  ros::Publisher pub_announce_, pub_contract_, pub_cancel_;
  ros::NodeHandle nh_, ph_;
  ros::Rate loop_rate_;
  ros::Timer periodic_func_;
  //
  std::string assigner_name_;
  double assigner_timeout_;

  bool cn_active_memory_,
       periodic_reassigment_enabled_,
       contractnet_in_process_,
       some_robot_bid_,
       some_robot_reject_to_bid_,
       request_heard_by_someone_,
       awarded_contract_rejected_,
       awarded_contract_accepted_,
       sim_monitor_request_,
       simulation_;

  int  num_robots_that_heard_req_, num_robots_that_bid_;

  std::vector<candidateStruct> candidates_list_;
  std::string lJobAssignedTo_;

  int key_file_descriptor_;
  struct termios original_terminal_state_;
  ecl::Thread thread_;

  char key_;

protected:
  void waiterRefuseCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  void waiterProposalCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  void waiterReceivedCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  void waiterAwardRespCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  //
  void jobAssignerBeh();
  void contractNet(robPosition pDesiredGoal);
  int awardContract(robPosition pDesiredGoal);
  std::string evaluateProposal(std::vector<candidateStruct> &candidates_list_, int nIndex=0);
  void resetAllMemories();
  bool waitUntilRobotsBidOrTimeout(double timeout);
  bool waitUntilRequest();
  bool waitUntil(bool &input);
  void printCandidateList(std::vector<candidateStruct> list);
  //
  void simMonitorRequest(const std_msgs::Bool::ConstPtr& msg);
  void periodicCallback(const ros::TimerEvent& event);
  //
  void sendAnnounceTaskMsg(robPosition pDesiredGoal);
  void sendAwardContractMsg(std::string stWinner,robPosition GoalPos);
  void sendCancelContractMsg(std::string stWinner);

  char getKey();
  void keyboardInputLoop();

  //
  int checkAwardStatus(int nIndex);
};

#endif /* ASSIGNERAGENT_H_ */
