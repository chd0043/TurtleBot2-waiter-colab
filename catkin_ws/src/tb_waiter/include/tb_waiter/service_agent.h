/*
 * ServiceAgent.h
 *
 *  Created on: Jun 30, 2014
 *    Author: chd
 */

#ifndef SERVICEAGENT_H_
#define SERVICEAGENT_H_

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <string>
#include <math.h>
#include <algorithm>
#include <time.h>

#include <ros/ros.h>
#include <tb_waiter/ContractNetMsg.h>
#include <tb_waiter/kobuki_command.h>
#include <tb_waiter/datalogger.hpp>

struct candidateStruct
{
  std::string emitter;
  RobPosition position;
  double distance;
  bool operator<(const candidateStruct& rhs) const { return (distance < rhs.distance); }
  candidateStruct():emitter(""),distance(0){}
};


class ServiceAgent
{
public:
  ServiceAgent();
  virtual ~ServiceAgent();

  void spin();

private:
  ros::Subscriber sub_announce_, sub_assign_, sub_cancel_;
  ros::Publisher pub_refusal_, pub_proposal_, pub_received_, pub_award_resp_;
  ros::NodeHandle nh_, ph_;
  ros::Rate loop_rate_;
  //
  std::string robot_name_;
  int max_num_aswer_;
  double sample_time_;

  ros::Time begin_time_ ;

  KobukiCommand kobuki_base_;
  RobPosition inicial_pose_;
  candidateStruct assigment_;

  DataLogger csv_logger_;
  dataStruct dataline_;

  bool robot_free_;

  std::vector<candidateStruct> candidate_list_;

  void mainProcess();
  void assignerAnnounceCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  void assignerContractCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  void assignerCancelCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg);
  //
  void evaluateProposal(int nMaxNumBid, std::vector<candidateStruct> &announce_heard_list);
  void sortByEuclideanDist(std::vector<std::string> &candidates_list);
  void sendRefuseMsg(std::string manager_name);
  void sendProposalMsg(std::string manager_name, double offer);

  void hearAnnoucement(std::string manager_name, std::string content);
  double estimateDistance(RobPosition position);

  void assignMsgBehav(std::string manager_name, std::string content);
  void sendReceivedFeedback(std::string manager_name);
  bool executeTask(RobPosition posAssigned);
  void goToHome(RobPosition pHome);

  void awardingResponseMsg(std::string manager_name, std::string response);
  RobPosition stringToPose(std::string input);
  //void stringToPose();
  void printCandidateList(std::vector<candidateStruct> list);
  bool waitTime(double time_preset);
  void evaluateCandidateHeard();

};
#endif /* SERVICEAGENT_H_ */
