/*
 * AssignerAgent.cpp
 *
 *  Created on: Jun 30, 2014
 *    Author: chd
 */

#include "tb_waiter/assigner_agent.h"


AssignerAgent::AssignerAgent() :
  nh_("~"),
  cn_active_memory_             (false),
  periodic_reassigment_enabled_ (false),
  contractnet_in_process_       (false),
  some_robot_bid_               (false),
  some_robot_reject_to_bid_     (false),
  request_heard_by_someone_     (false),
  awarded_contract_rejected_    (false),
  awarded_contract_accepted_    (false),
  sim_monitor_request_          (false),
  num_robots_that_heard_req_    (0),
  num_robots_that_bid_          (0),
  key_file_descriptor_          (0),
  loop_rate_                    (10),
  key_                          ('\0')
{
  sub_refusal_    = nh_.subscribe("/cn_waiter_refuse_msg",     10, &AssignerAgent::waiterRefuseCallback,    this);
  sub_proposal_   = nh_.subscribe("/cn_waiter_proposal_msg",   10, &AssignerAgent::waiterProposalCallback,  this);
  sub_received_   = nh_.subscribe("/cn_waiter_received_msg",   10, &AssignerAgent::waiterReceivedCallback,  this);
  sub_award_resp_ = nh_.subscribe("/cn_waiter_award_resp_msg", 10, &AssignerAgent::waiterAwardRespCallback, this);
  sub_sim_coord_  = ph_.subscribe("sim_coordinator",           10, &AssignerAgent::simMonitorRequest,       this);

  pub_announce_ = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_assigner_announce", 1);
  pub_contract_ = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_assigner_contract", 1);
  pub_cancel_   = nh_.advertise<tb_waiter::ContractNetMsg>("/cn_assigner_cancel",   1);

  periodic_func_ = nh_.createTimer(ros::Duration(5.0), &AssignerAgent::periodicCallback, this);

  nh_.param("assigner_name", assigner_name_, std::string("-"));
  nh_.param("assigner_timeout", assigner_timeout_, 30.0);
  nh_.param("simulation", simulation_, false); //true

  tcgetattr(key_file_descriptor_, &original_terminal_state_); // get terminal properties
  thread_.start(&AssignerAgent::keyboardInputLoop, *this);

  ROS_INFO("Assigner agent initialized.");
  ROS_INFO("Waiting for signal.");
}

AssignerAgent::~AssignerAgent()
{
  //thread_.cancel();
  tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
}

void AssignerAgent::spin()
{
  while (ros::ok())
  {
    jobAssignerBeh();
    ros::spinOnce();
    loop_rate_.sleep();
    //ROS_INFO("une: %c", key_);
  }
  //thread_.join();
  thread_.cancel();
  tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
}

void AssignerAgent::jobAssignerBeh()
{
  robPosition goal;

  if    (assigner_name_ == "table0"){
    goal.x = -3.280; goal.y = 4.460; }
    //goal.x = -3.2753636837; goal.y =  2.30509614944; }
  else if (assigner_name_ == "table1") {
    //goal.x = -5.08998632431; goal.y =  -0.326577186584; }
    goal.x = -2.410; goal.y = 1.010; }
  else if (assigner_name_ == "table2") {
    goal.x = -2.492; goal.y =-1.260; }
  else if (assigner_name_ == "table3") {
    goal.x = -4.161; goal.y = 5.884; }

  if ( waitUntilRequest() && lJobAssignedTo_.empty() )
  {
    cn_active_memory_ = true;
    periodic_reassigment_enabled_ = false;
    contractNet(goal);
    ros::Duration(1.0).sleep();
    while(!lJobAssignedTo_.empty() and ros::ok()){ros::spinOnce();}
    ROS_INFO(" --- Waiting for signal!");
  }
}

void AssignerAgent::contractNet(robPosition pDesiredGoal)
{
  int step = 1;
  enum steps {STP_0, STP_1, STP_2, STP_3, STP_4};

  while ( step <= 4 && step >= 1 && ros::ok() )
  {
    contractnet_in_process_ = true;

    switch( step )
    {
    case STP_1: // ### step 1 ###
      ROS_INFO("step 1.");
      resetAllMemories();
      candidates_list_.clear();
      step = 2;
      break;

    case STP_2: // ### step 2 ###
      ROS_INFO("step 2.");
      sendAnnounceTaskMsg(pDesiredGoal);
      step = 3;
      break;

    case STP_3: // ### step 3 ###
      ROS_INFO("step 3.");
      if ( waitUntilRobotsBidOrTimeout(assigner_timeout_) )
      {
        step = 0; // 1
        ROS_ERROR("Error. No one answer the CNET request.");
        //return;
      }
      else
        step = 4;
      break;

    case STP_4: // ### step 4 ###
      ROS_INFO("step 4.");
      printCandidateList(candidates_list_);
      step = awardContract(pDesiredGoal);
      break;

    default:
      step = 0;
      ROS_ERROR("Error. Unknown step.");
    }
  }
  contractnet_in_process_ = false;
}

int AssignerAgent::awardContract(robPosition pDesiredGoal)
{
  int step(0);
  bool job_assigned(false);
  int nIndex(0);
  enum award_status {AWARD_REJECTED, AWARD_ACCEPTED};
  std::string winner("");

  if ( some_robot_bid_  && !candidates_list_.empty() )
  {
    while ( !job_assigned && ros::ok( ))
    {
      winner = evaluateProposal(candidates_list_, nIndex);
      ROS_INFO("winner: %s", winner.c_str());
      sendAwardContractMsg(winner, pDesiredGoal);

      switch ( checkAwardStatus(nIndex) )
      {
      case AWARD_REJECTED:
        ROS_INFO("%s: winner denied.",winner.c_str());
        nIndex++;
        job_assigned = false;
        break;

      case AWARD_ACCEPTED:
        ROS_INFO("%s: winner accept.",winner.c_str());
        lJobAssignedTo_ = winner;
        job_assigned = true;
        step = 5;
        break;

      default:
        job_assigned = true;
        step = 1;
      }

      resetAllMemories();
      ros::spinOnce();
    }
  }
  else if (some_robot_reject_to_bid_)
  {
    job_assigned = true;
    step = 1;
  }
  return (step);
}

std::string AssignerAgent::evaluateProposal(std::vector<candidateStruct> &candidates_list_, int nIndex)
{
  std::string winner("none");
  std::sort(candidates_list_.begin(), candidates_list_.end());
  try
  {
    winner = candidates_list_[nIndex].emitter;
  }
  catch (const std::out_of_range& e)
  {
    ROS_ERROR("Vector 'candidates_list_' out of range: %s", e.what() );
  }
  return (winner);
}

void AssignerAgent::resetAllMemories()
{
  key_                       = '\0';
  some_robot_bid_            = false;
  some_robot_reject_to_bid_  = false;
  awarded_contract_accepted_ = false;
  awarded_contract_rejected_ = false;
  request_heard_by_someone_  = false;
  num_robots_that_heard_req_ = 0;
  num_robots_that_bid_       = 0;
}

int AssignerAgent::checkAwardStatus(int nIndex)
{
  int state;
  float time_count(0);
  float timeout(1.0);
  enum award_status {AWARD_REJECTED, AWARD_ACCEPTED, NONE};

  awarded_contract_rejected_ = false;
  awarded_contract_accepted_ = false;

  while ( !(awarded_contract_rejected_ || awarded_contract_accepted_) && !(time_count >= timeout) )
  {
    ros::Duration(0.1).sleep();
    time_count+=0.1;
    ros::spinOnce();
  }

  if ( awarded_contract_rejected_ && candidates_list_.size() > (nIndex+1) )
  {
    ROS_DEBUG("awarded_contract_rejected_");
    state = AWARD_REJECTED;
  }
  else if ( time_count >= timeout && candidates_list_.size() > (nIndex+1) )
  {
    ROS_DEBUG("time_count >= timeout");
    state = AWARD_REJECTED;
  }
  else if (awarded_contract_accepted_ )
  {
    ROS_DEBUG("awarded_contract_accepted_");
    state = AWARD_ACCEPTED;
  }
  else
  {
    state = NONE;
  }
  return (state);
}

bool AssignerAgent::waitUntilRequest()
{
  if (!simulation_)
  {
    if (sim_monitor_request_)
    {
      sim_monitor_request_ = false;
      return(true);
    }
    else
      return(false);
  }
  else
  {
    char key = getKey();
    if (key == '\0')
      return (false);
    else
    {
      if (key == '\x03') ros::shutdown();//exit(EXIT_SUCCESS);
      return (true);
    }
  }
}

char AssignerAgent::getKey()
{
  char resp;
  resp = key_;
  key_ = (char)0;
  return(resp);
}

void AssignerAgent::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor_, TCSANOW, &raw);

  while (ros::ok())
  {
    if (read(key_file_descriptor_, &key_, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    ROS_INFO("key_: %c", key_);
  }
  exit(EXIT_SUCCESS);
}

bool AssignerAgent::waitUntilRobotsBidOrTimeout(double timeout)
{
  double time_count(0);
  while ( !( ( (num_robots_that_heard_req_ == num_robots_that_bid_) && (some_robot_bid_ || some_robot_reject_to_bid_) )
          || (time_count >= timeout) )
          && ros::ok() )
  {
    ros::Duration(0.1).sleep();
    time_count+=0.1;
    ros::spinOnce();
  }
  return( !(some_robot_bid_ || some_robot_reject_to_bid_) );
}

bool AssignerAgent::waitUntil(bool &input)
{
    while(input && ros::ok)
    {
        ros::spinOnce();
    }
return (true);
}


void AssignerAgent::printCandidateList(std::vector<candidateStruct> list)
{
// Prints candidate list in format:
/*
 Candidate List:
['robot_0', 7.74]
['robot_1', 3.85]
 */
  std::stringstream string;
  if ( list.empty() ) return;

  for (std::vector<candidateStruct>::iterator it_list = list.begin() ; it_list != list.end(); ++it_list)
  {
      string << "['"<< (*it_list).emitter << "', "
                    << (*it_list).offer   << "] \n";
  }
  ROS_INFO("\n\n Candidate List:\n%s", string.str().c_str());
}

void AssignerAgent::sendAnnounceTaskMsg(robPosition pDesiredGoal)
{
  tb_waiter::ContractNetMsg msg;

  std::stringstream posString;
  posString << pDesiredGoal.x<<","<< pDesiredGoal.y;

  msg.setPerformative = "inform";
  msg.sender = assigner_name_;
  msg.receiver = "all";
  msg.content = posString.str();
  msg.language = "s1";
  msg.ontology = "posAnnouncement";

  pub_announce_.publish(msg);
  ROS_INFO("%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                   msg.content.c_str(), msg.receiver.c_str());
}

void AssignerAgent::sendAwardContractMsg(std::string stWinner, robPosition GoalPos)
{
  tb_waiter::ContractNetMsg msg;

  std::stringstream posString;
  posString << GoalPos.x<<","<< GoalPos.y;

  msg.setPerformative = "inform";
  msg.sender = assigner_name_;
  msg.receiver = stWinner;
  msg.content = posString.str();
  msg.language = "s1";
  msg.ontology = "AwardPosition";

  pub_contract_.publish(msg);
  ROS_INFO("%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                   msg.content.c_str(), msg.receiver.c_str());
}

void AssignerAgent::sendCancelContractMsg(std::string stWinner)
{
  tb_waiter::ContractNetMsg msg;

  msg.setPerformative = "inform";
  msg.sender = assigner_name_;
  msg.receiver = "all";
  msg.content = "cancel";
  msg.language = "s1";
  msg.ontology = "cancelContract";

  pub_cancel_.publish(msg);
  ROS_INFO("%s send %s: %s, to %s",msg.sender.c_str(),  msg.ontology.c_str(),
                                   msg.content.c_str(), msg.receiver.c_str());
}
// ##################################################### //

void AssignerAgent::simMonitorRequest(const std_msgs::Bool::ConstPtr& msg)
{
  sim_monitor_request_ = msg->data; //true;
  ROS_INFO("sim_monitor_request_: %i",sim_monitor_request_);
}

void AssignerAgent::periodicCallback(const ros::TimerEvent& event)
{
  periodic_reassigment_enabled_ = true;
}

// ##################################################### //

void AssignerAgent::waiterRefuseCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == assigner_name_ || (msg->receiver) == "all")
  {
    some_robot_reject_to_bid_ = true;
    num_robots_that_bid_ += 1;
    //
    ROS_INFO("Received from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                     msg->receiver.c_str());
  }

}

void AssignerAgent::waiterProposalCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == assigner_name_ || (msg->receiver) == "all")
  {
    candidateStruct newCandidate;

    newCandidate.emitter = msg->sender;;
    newCandidate.offer = atof(msg->content.c_str());
    candidates_list_.push_back(newCandidate);
    some_robot_bid_ = true;
    num_robots_that_bid_ += 1;
    //
    ROS_INFO("Received from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                     msg->receiver.c_str());
  }
}

void AssignerAgent::waiterReceivedCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == assigner_name_ || (msg->receiver) == "all")
  {
    request_heard_by_someone_ = true;
    num_robots_that_heard_req_ += 1;
    ROS_INFO("Received from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                     msg->receiver.c_str());
  }
}

void AssignerAgent::waiterAwardRespCallback(const tb_waiter::ContractNetMsg::ConstPtr& msg)
{
  if ( (msg->receiver) == assigner_name_ || (msg->receiver) == "all")
  {
    std::string stContent = msg->content;
    if (stContent == "Job_Accepted")
      awarded_contract_accepted_ = true;
    if (stContent == "Job_Rejected")
      awarded_contract_rejected_ = true;
    if (stContent == "Job_Finish")
      cn_active_memory_ = false;
      lJobAssignedTo_.clear();
      //ROS_INFO("Waiting for signal.");
    if (stContent == "Job_Fail")
      lJobAssignedTo_.clear();
    //
    ROS_INFO("Received from: %s, content: %s; to %s",msg->sender.c_str(),  msg->content.c_str(),
                                                     msg->receiver.c_str());
  }
}

// ##################################################### //
// ##################################################### //
// ##################################################### //
// ##################################################### //

int main(int argc, char **argv)
{
  ros::init(argc, argv, "assigner_agent");

  AssignerAgent Agent;
  Agent.spin();

  return (0);
}

