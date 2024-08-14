/*
 * ServiceAgent.cpp
 *
 *  Created on: Jun 30, 2014
 *    Author: chd
 */

#include "tb_waiter/service_agent.h"

// ############################################################################################# //
// ##################################### ServiceAgent ########################################## //
// ############################################################################################# //

ServiceAgent::ServiceAgent():
  ph_("~"),
  loop_rate_  (10),
  robot_free_ (true)
{
  // constructor
  sub_announce_ = nh_.subscribe("/cn_assigner_announce", 10, &ServiceAgent::assignerAnnounceCallback, this);
  sub_assign_   = nh_.subscribe("/cn_assigner_contract", 10, &ServiceAgent::assignerContractCallback, this);
  sub_cancel_   = nh_.subscribe("/cn_assigner_cancel",   10, &ServiceAgent::assignerCancelCallback,   this);

  pub_refusal_    = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_waiter_refuse_msg",    1);
  pub_proposal_   = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_waiter_proposal_msg",  1);
  pub_received_   = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_waiter_received_msg",  1);
  pub_award_resp_ = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_waiter_award_resp_msg",1);

  ph_.param("robot_name",      robot_name_,    std::string("kobuki_base__0"));
  ph_.param("service_timeout", sample_time_,   2.0);
  ph_.param("max_num_aswer",   max_num_aswer_, 2);

  csv_logger_.initLogFile(robot_name_);
  //
  ROS_INFO("Service agent initialized.");
}

ServiceAgent::~ServiceAgent()
{
  // Auto-generated destructor stub
  ROS_INFO("Service agent destroyed");
}

// ############################################################################################# //

void ServiceAgent::spin()
{
  // reference
  kobuki_base_.rAutodocking();
  inicial_pose_ = kobuki_base_.getPosActual();

  while (ros::ok())
  {
    //ROS_INFO("hearing...");
    mainProcess();
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// ############################################################################################# //

void ServiceAgent::printCandidateList(std::vector<candidateStruct> list)
{
// Prints candidate list in format:
/*
 Candidate List:
['table0',(-3.28, 4.46), 7.74]
['table1',(-3.03, 0.82), 3.85]
 */
  std::stringstream string;
  if ( list.empty() ) return;

  for (std::vector<candidateStruct>::iterator it_list = list.begin() ; it_list != list.end(); ++it_list)
  {
      string << "['"<< (*it_list).emitter    << "',("
                    << (*it_list).position.x << ", "
                    << (*it_list).position.y << "), "
                    << (*it_list).distance   << "] \n";
  }
  ROS_INFO("\n\n Candidate List:\n%s", string.str().c_str());
}

bool ServiceAgent::waitTime(double time_preset)
{
  double timeout;
  while (timeout < time_preset && ros::ok())
  {
      ros::Duration(0.1).sleep(); //sample_time_
      timeout +=0.1;
      ros::spinOnce();
  }
  return(true);
}

void ServiceAgent::mainProcess()
{
  evaluateCandidateHeard();

  if ( !robot_free_ )
  {
    bool status = executeTask(assigment_.position);
    if (status)
      awardingResponseMsg(assigment_.emitter,"Job_Finish");
    else
      awardingResponseMsg(assigment_.emitter,"Job_Fail");
    assigment_ = candidateStruct();
    goToHome(inicial_pose_);
    robot_free_ = true;
  }
}

void ServiceAgent::evaluateCandidateHeard()
{
  RobPosition pActualPos;

  if (candidate_list_.size() > 0)
  {
    //ros::Duration(5.0).sleep(); //sample_time_
    waitTime(sample_time_);
    pActualPos = kobuki_base_.getPosActual();
    printCandidateList(candidate_list_);
    ROS_INFO("pActualPos: (%f, %f)", pActualPos.x, pActualPos.y);
    evaluateProposal(max_num_aswer_, candidate_list_);
  }

}

void ServiceAgent::evaluateProposal(int nMaxNumBid, std::vector<candidateStruct> &announce_heard_list)
{
  if ( announce_heard_list.empty() )
     return;
  std::string manager_name("-");
  int i=0;
  double offer;
  std::sort(announce_heard_list.begin(), announce_heard_list.end());

  for (std::vector<candidateStruct>::iterator it = announce_heard_list.begin() ; it != announce_heard_list.end(); ++it, ++i)
  {
    manager_name = (*it).emitter;
    if ( i < nMaxNumBid){
      ROS_INFO("Accepted, %i",i);

      try
      {
        offer = kobuki_base_.estimateCost( (*it).position) ;
        sendProposalMsg(manager_name, offer);
      }
      catch (const std::out_of_range& e)
      {
        ROS_ERROR("Vector 'announce_heard_list' out of range: %s", e.what() );
      }

    }
    else{
      ROS_INFO("Rejected, %i",i);
      //ROS_INFO("it, %i",it);
      ROS_INFO("size: %d", (int)announce_heard_list.size());
      sendRefuseMsg(manager_name);
    }
    ros::spinOnce();
  }
  announce_heard_list.clear();
}

void ServiceAgent::sendRefuseMsg(std::string manager_name)
{
  if ( manager_name.empty() )
      return;
  tb_waiter::ContractNetMsg msg;

  msg.setPerformative = "inform";
  msg.sender = robot_name_;
  msg.receiver = manager_name;
  msg.content = "refuse";
  msg.language = "s1";
  msg.ontology = "rob_refuse";

  pub_refusal_.publish(msg);
  ROS_INFO("\n%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                     msg.content.c_str(), msg.receiver.c_str());
}

void ServiceAgent::sendProposalMsg(std::string manager_name, double offer)
{
  tb_waiter::ContractNetMsg msg;

  std::stringstream stOffer;
  stOffer << offer;

  msg.setPerformative = "inform";
  msg.sender = robot_name_;
  msg.receiver = manager_name;
  msg.content = stOffer.str();
  msg.language = "s1";
  msg.ontology = "rob_proposal";

  pub_proposal_.publish(msg);
  ROS_INFO("\n%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                     msg.content.c_str(), msg.receiver.c_str());
}

// ############################################################################################# //

void ServiceAgent::hearAnnoucement(std::string manager_name, std::string content)
{
  if ( content.empty() || manager_name.empty() )
  {
    ROS_ERROR("content or manager_name is empty. (191)");
    return;
  }

  candidateStruct new_candidate;

  RobPosition pose_assigned = stringToPose(content);
  double dist = estimateDistance(pose_assigned);

  new_candidate.emitter  = manager_name;
  new_candidate.position = pose_assigned;
  new_candidate.distance = dist;

  candidate_list_.push_back(new_candidate);
  sendReceivedFeedback(manager_name);
}

double ServiceAgent::estimateDistance(RobPosition position)
{
  double dist,xd,yd;
  RobPosition pActualPos = kobuki_base_.getPosActual();
  xd = pActualPos.x - position.x;
  yd = pActualPos.y - position.y;
  //
  dist = fabs(xd) + fabs(yd);
  return (dist);
}

void ServiceAgent::sendReceivedFeedback(std::string manager_name)
{
  if ( manager_name.empty() )
  {
    ROS_ERROR("manager_name is empty. (223)");
    return;
  }

  tb_waiter::ContractNetMsg msg;

  msg.setPerformative = "inform";
  msg.sender = robot_name_;
  msg.receiver = manager_name;
  msg.content = "processing request";
  msg.language = "s1";
  msg.ontology = "request_received";

  pub_received_.publish(msg);
  ROS_INFO("\n%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                     msg.content.c_str(), msg.receiver.c_str());
}

// ############################################################################################# //
void ServiceAgent::assignMsgBehav(std::string manager_name, std::string content)
{
  if ( content.empty() || manager_name.empty() )
  {
    ROS_ERROR("content or manager_name is empty. (246)");
    return;
  }
  bool status;
  RobPosition pose_assigned = stringToPose(content);
  dataline_.winner = manager_name;
  dataline_.position = content;

  if ( robot_free_ )
  {
    ROS_INFO("Assigned: %s", manager_name.c_str());
    awardingResponseMsg(manager_name,"Job_Accepted");
    robot_free_ = false;

    assigment_.emitter = manager_name;
    assigment_.position = pose_assigned;
  }
  else
  {
    ROS_WARN("Job_Rejected");
    awardingResponseMsg(manager_name,"Job_Rejected");
  }
}

bool ServiceAgent::executeTask(RobPosition pose_assigned)
{
  ROS_INFO(" %s: Assigned position -> x:%f , y:%f.",robot_name_.c_str(), pose_assigned.x, pose_assigned.y);

  dataline_.distance = kobuki_base_.getMeasuredDistance();
  begin_time_ = ros::Time::now();

  kobuki_base_.sendMoveBaseGoal(pose_assigned);  // navigateToGoal
  while (!kobuki_base_.getMoveBaseDoneStatus() && ros::ok())
  {
    evaluateCandidateHeard();
    ros::spinOnce();
  }

  bool status = true;
  pose_assigned= RobPosition();

  return(status);
}

void ServiceAgent::goToHome(RobPosition pHome)
{
  kobuki_base_.navigateToGoal(pHome);
  kobuki_base_.clearCostmaps();

  // ### store data log ### /
  ros::Duration cicle_time = ( ros::Time::now() - begin_time_ );
  double dist2 = kobuki_base_.getMeasuredDistance();
  dataline_.time = cicle_time.toSec();
  dataline_.distance = fabs(dist2 - dataline_.distance);

  ROS_INFO("Distance traveled: %f.", dataline_.distance);
  ROS_INFO("Cycle Time: %f.", cicle_time.toSec());

  csv_logger_.addline(dataline_);
  dataline_ = dataStruct();
}

void ServiceAgent::awardingResponseMsg(std::string manager_name, std::string response)
{
  tb_waiter::ContractNetMsg msg;

  msg.setPerformative = "inform";
  msg.sender = robot_name_;
  msg.receiver = manager_name;
  msg.content = response;
  msg.language = "s1";
  msg.ontology = "robAwardResponce";

  pub_award_resp_.publish(msg);
  ROS_INFO("\n%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                     msg.content.c_str(), msg.receiver.c_str());
}

// ############################################################################################# //

RobPosition ServiceAgent::stringToPose(std::string input)
{
  RobPosition pos;
  std::vector<double> val;
  std::istringstream ss(input);
  std::string token;

  for (int i; std::getline(ss, token, ','); i++ )
  {
    val.push_back(atof(token.c_str()));
  }
  if (val.size() > 1){
    pos.x = val[0];
    pos.y = val[1];
  }
  return (pos);
}

// ############################################################################################# //

void ServiceAgent::assignerAnnounceCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == robot_name_ || (msg->receiver) == "all")
  {
    ROS_INFO("\nReceived from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                       msg->receiver.c_str());
    //if ( robot_free_ )
    hearAnnoucement((msg->sender), (msg->content));
  }
}

void ServiceAgent::assignerContractCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == robot_name_ || (msg->receiver) == "all")
  {
    ROS_INFO("\nReceived from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                       msg->receiver.c_str());
    assignMsgBehav( (msg->sender), (msg->content) );
  }
}

void ServiceAgent::assignerCancelCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == robot_name_ || (msg->receiver) == "all")
  {
    ROS_INFO("\nReceived from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                       msg->receiver.c_str());
    kobuki_base_.cancelGoal();
  }
}
// ############################################################################################# //
// ############################################################################################# //
// ############################################################################################# //

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_agent");
  ServiceAgent Agent;
  Agent.spin();

  //ros::init(argc, argv, "MyNode", ros::init_options::NoSigintHandler);
  //signal(SIGINT, mySigIntHandler);

  return (0);
}

