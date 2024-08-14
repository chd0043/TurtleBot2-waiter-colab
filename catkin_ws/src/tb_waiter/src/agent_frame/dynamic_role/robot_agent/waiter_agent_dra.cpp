/*
 * waiter_agent_dra.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: chd
 */

#include <tb_waiter/waiter_agent_dra.h>

WaiterAgentDra::WaiterAgentDra():
  ph_              ("~"),
  loop_rate_       (10),
  bRobotFree_      (true),
  dra_state_       (ST0_IDLE),
  current_utility_ (0)
{
  ph_.param("robot_name", robot_name_, std::string("robot_0"));
  ph_.param("simulation", simulation_, true);

  sub_announce_t0_   = nh_.subscribe("/table0/dra_announce", 10, &WaiterAgentDra::table0AnnounceCallback, this);
  sub_announce_t1_   = nh_.subscribe("/table1/dra_announce", 10, &WaiterAgentDra::table1AnnounceCallback, this);
  sub_announce_t2_   = nh_.subscribe("/table2/dra_announce", 10, &WaiterAgentDra::table2AnnounceCallback, this);
  sub_announce_t3_   = nh_.subscribe("/table3/dra_announce", 10, &WaiterAgentDra::table3AnnounceCallback, this);
  sub_announce_t4_   = nh_.subscribe("/table4/dra_announce", 10, &WaiterAgentDra::table4AnnounceCallback, this);
  //
  sub_dra_status_r0_ = nh_.subscribe("/robot_0/dra_status",  10, &WaiterAgentDra::robot0StatusCallback,   this);
  sub_dra_status_r1_ = nh_.subscribe("/robot_1/dra_status",  10, &WaiterAgentDra::robot1StatusCallback,   this);
  sub_dra_status_r2_ = nh_.subscribe("/robot_2/dra_status",  10, &WaiterAgentDra::robot2StatusCallback,   this);
  sub_dra_status_r3_ = nh_.subscribe("/robot_3/dra_status",  10, &WaiterAgentDra::robot3StatusCallback,   this);
  sub_dra_status_r4_ = nh_.subscribe("/robot_4/dra_status",  10, &WaiterAgentDra::robot4StatusCallback,   this);

  periodic_func_   = nh_.createTimer(ros::Duration(1.0), &WaiterAgentDra::periodicCallback, this);

  pub_dra_status_  = nh_.advertise<tb_waiter::DynamicRoleInfo>("dra_status",5);

  tables_req_service_.resize(5);
  dra_status_list_.resize(5);

  csv_logger_.initLogFile(robot_name_,"2");
  //
  ROS_INFO("Waiter agent initialized.");
}
WaiterAgentDra::~WaiterAgentDra()
{
  ROS_ERROR("Adios");
}

// ############################################################################################# //

void WaiterAgentDra::spin()
{
  // reference
  kobuki_base_.rAutodocking();
  initial_pose_ = kobuki_base_.getPosActual();

  while (ros::ok())
  {
    //ROS_INFO("hearing...");
    finiteStateMachine();
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// ############################################################################################# //

void WaiterAgentDra::fsmSt0Idle()
{
  ROS_INFO("ST0_IDLE");
  current_state_ = "ST0_IDLE";
  dra_state_ = lookupTransitions(ST0_IDLE);
}

void WaiterAgentDra::fsmSt1Dock()
{
  ROS_INFO("ST1_DOCK");
  current_state_ = "ST1_DOCK";
  kobuki_base_.rAutodocking();
  initial_pose_ = kobuki_base_.getPosActual();
  dra_state_ = lookupTransitions(ST1_DOCK);
}

void WaiterAgentDra::fsmSt2Waiting()
{
  ROS_INFO("ST2_WAITING");
  current_state_ = "ST2_WAITING";
  dra_state_ = lookupTransitions(ST2_WAITING);
}

void WaiterAgentDra::fsmSt3Deliver()
{
  ROS_INFO("ST3_DELIVER");
  current_state_ = "ST3_DELIVER";
  RobPosition goal, current_goal, zero_pose ;

  dataline_.distance = kobuki_base_.getMeasuredDistance();
  begin_time_ = ros::Time::now();

  do
  {
    goal = selectTable(tables_req_service_);
    if ( ( goal != current_goal) )
      kobuki_base_.sendMoveBaseGoal(goal);

    current_goal = goal;
    current_goal_ = current_goal;
    ros::spinOnce();
  }
  while ( !kobuki_base_.getMoveBaseDoneStatus() && (goal != zero_pose) && ros::ok() ); //

  if (goal != zero_pose)
    current_state_ = "deliver_ok";
  kobuki_base_.clearCostmaps();

  // datalog info
  // # # #
  std::ostringstream ss_position;
  ss_position << current_goal.x <<","<< current_goal.y;
  dataline_.winner = assigned_table_;
  dataline_.position = ss_position.str();
  // # # #

  tables_req_service_.clear(); tables_req_service_.resize(5);
  waitTime(2.0);
  dra_state_ = lookupTransitions(ST3_DELIVER);
}

void WaiterAgentDra::fsmSt4Home()
{
  ROS_INFO("ST4_HOME");
  current_state_ = "ST4_HOME";
  kobuki_base_.navigateToGoal(initial_pose_);

  // ### store datalog info ###
  ros::Duration cicle_time = ( ros::Time::now() - begin_time_ );
  double dist2 = kobuki_base_.getMeasuredDistance();
  dataline_.time = cicle_time.toSec();
  dataline_.distance = fabs(dist2 - dataline_.distance);

  ROS_INFO("Distance traveled: %f.", dataline_.distance);
  ROS_INFO("Cycle Time: %f.", cicle_time.toSec());

  csv_logger_.addline(dataline_);
  dataline_ = dataStruct();
  // ####

  kobuki_base_.clearCostmaps();

  dra_state_ = lookupTransitions(ST4_HOME);
}

int WaiterAgentDra::lookupTransitions(int cur_state)
{
  int next_state;

  switch (cur_state)
  {
  case ST0_IDLE:
    next_state = ST1_DOCK;
    break;
  case ST1_DOCK:
    next_state = ST2_WAITING;
    break;
  case ST2_WAITING:
    waitForRequest();
    next_state = ST3_DELIVER;
    break;
  case ST3_DELIVER:
    next_state = ST4_HOME;
    break;
  case ST4_HOME:
    next_state = ST2_WAITING;
    break;
  }

  return(next_state);
}

void WaiterAgentDra::finiteStateMachine()
{
  switch ( dra_state_ )
  {
  case ST0_IDLE:    fsmSt0Idle();     break;
  case ST1_DOCK:    fsmSt1Dock();     break;
  case ST2_WAITING: fsmSt2Waiting();  break;
  case ST3_DELIVER: fsmSt3Deliver();  break;
  case ST4_HOME:    fsmSt4Home();     break;
  default: {}
  }
}

// ############################################################################################# //

bool WaiterAgentDra::waitForRequest()
{
  bool request_exist(false);
  while ( !request_exist && ros::ok() )
  {
    if ( !tables_req_service_.empty() )
    {
      for (std::vector<draRequestStruct>::iterator it_table  = tables_req_service_.begin() ;
                                                   it_table != tables_req_service_.end(); ++it_table)
      {
        ros::spinOnce();
        if ( it_table->status   == "request" &&
             it_table->contract == robot_name_ )
          request_exist = true;
      }
    }
    ros::spinOnce();
  }
  return (request_exist);
}

/*
RobPosition WaiterAgentDra::selectTable(std::vector<draRequestStruct> tables_req_service)
{
  RobPosition pose;
  if ( tables_req_service.empty() )
  {
    ROS_ERROR("Variable 'tables_req_service_' is empty.");
    return( kobuki_base_.getPosActual() );
  }

  int last = tables_req_service.size() - 1;
  std::sort(tables_req_service.begin(), tables_req_service.end());
  //std::distance(tables_req_service, max_element( tables_req_service.begin(), tables_req_service.end() ) );
  pose.x = tables_req_service[last].pose.pose.position.x;
  pose.y = tables_req_service[last].pose.pose.position.y;
  assigned_table_ = tables_req_service[last].name;
  return(pose);
}
*/

RobPosition WaiterAgentDra::selectTable(std::vector<draRequestStruct> tables_req_service)
{
  draRequestStruct best_candidate;
  RobPosition blank_pose, goal = initial_pose_;
  int robot_num_id(0), last_robot_num_id(-99);

  for (std::vector<draRequestStruct>::iterator it_table  = tables_req_service.begin() ;
                                               it_table != tables_req_service.end(); ++it_table, robot_num_id++)
  {

    ros::spinOnce();
    if ( it_table->status   == "request"    &&
         it_table->contract == robot_name_  &&
         it_table->utility  >  best_candidate.utility )
    {
      best_candidate = *it_table;
    }
  }
  goal.x = best_candidate.pose.pose.position.x;
  goal.y = best_candidate.pose.pose.position.y;
  assigned_table_  = best_candidate.name;
  current_utility_ = best_candidate.utility;
  //}
  //std::cout << "goal: " << goal.x << ", " << goal.y << std::endl;
  return(goal);
}

bool WaiterAgentDra::waitTime(double time_preset)
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

// ############################################################################################# //

double WaiterAgentDra::estimateDistance(geometry_msgs::PoseWithCovariance position)
{
  double dist,xd,yd;
  RobPosition pActualPos = kobuki_base_.getPosActual();
  xd = pActualPos.x - position.pose.position.x;
  yd = pActualPos.y - position.pose.position.y;
  //
  dist = fabs(xd) + fabs(yd);
  return (dist);
}

double WaiterAgentDra::estimateDistance(RobPosition position)
{
  double dist,xd,yd;
  RobPosition pActualPos = kobuki_base_.getPosActual();
  xd = pActualPos.x - position.x;
  yd = pActualPos.y - position.y;
  //
  dist = fabs(xd) + fabs(yd);
  return (dist);
}

double WaiterAgentDra::estimateDistance(RobPosition position1, RobPosition position2)
{
  double dist,xd,yd;
  xd = position1.x - position2.x;
  yd = position1.y - position2.y;
  //
  dist = fabs(xd) + fabs(yd);
  return (dist);
}

// ############################################################################################# //

double WaiterAgentDra::estimateIdleStateUtility()
{
    return (0);
}
double WaiterAgentDra::estimateDockerStateUtility()
{
    return (0);
}
double WaiterAgentDra::estimateWaiterStateUtility()
{
    return (0);
}
double WaiterAgentDra::estimateHomeStateUtility()
{
    return (0);
}
double WaiterAgentDra::estimateDeliverStateUtility(int table_id_num)
{
    //
  float utility(0), penalty(0);

  if ( (dra_state_ == ST3_DELIVER  || dra_state_ == ST2_WAITING ) &&
       tables_req_service_[table_id_num].status == "request" )
  {
    if ( dra_state_ == ST3_DELIVER && assigned_table_ != tables_req_service_[table_id_num].name )
      penalty = current_utility_ + estimateDistance(initial_pose_, current_goal_);

    utility = ( MAX_U - estimateDistance(tables_req_service_[table_id_num].pose) - penalty);
  }
  else
  {
    utility = 0;
  }
  return (utility);
}

void WaiterAgentDra::periodicCallback(const ros::TimerEvent& event)
{
  //ROS_INFO("Periodic... ");

  dra_my_status_.header.stamp = ros::Time::now();
  dra_my_status_.header.frame_id = robot_name_;

  dra_my_status_.robot_name = robot_name_;
  dra_my_status_.current_state =  current_state_;
  dra_my_status_.assigned_table = assigned_table_;
  dra_my_status_.current_utility = current_utility_;
  // state utilities
  dra_my_status_.u_idle   = estimateIdleStateUtility();
  dra_my_status_.u_docker = estimateDockerStateUtility();
  dra_my_status_.u_waiter = estimateWaiterStateUtility();
  dra_my_status_.u_home   = estimateHomeStateUtility();
  // table utility
  dra_my_status_.u_table[TABLE0] = tables_req_service_[TABLE0].utility;
  dra_my_status_.u_table[TABLE1] = tables_req_service_[TABLE1].utility;
  dra_my_status_.u_table[TABLE2] = tables_req_service_[TABLE2].utility;
  dra_my_status_.u_table[TABLE3] = tables_req_service_[TABLE3].utility;
  dra_my_status_.u_table[TABLE4] = tables_req_service_[TABLE4].utility;
  pub_dra_status_.publish(dra_my_status_);
}

//
void WaiterAgentDra::tableAnnounceAddToList(int table_id_num, const tb_waiter::DynamicRoleAssigment::ConstPtr& msg)
{
  tables_req_service_[table_id_num].table_id_num = table_id_num;
  tables_req_service_[table_id_num].name         = msg->name;
  tables_req_service_[table_id_num].type         = msg->type;
  tables_req_service_[table_id_num].status       = msg->status;
  tables_req_service_[table_id_num].contract     = msg->contract;
  tables_req_service_[table_id_num].pose         = msg->pose;
  tables_req_service_[table_id_num].utility      = estimateDeliverStateUtility(table_id_num);
  //ROS_INFO("hola: %d, utility: %f", table_num_id, tables_req_service_[table_num_id].utility);
}

void WaiterAgentDra::table0AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg)
{
  tableAnnounceAddToList(TABLE0, msg);
}

void WaiterAgentDra::table1AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg)
{
  tableAnnounceAddToList(TABLE1, msg);
}

void WaiterAgentDra::table2AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg)
{
  tableAnnounceAddToList(TABLE2, msg);
}

void WaiterAgentDra::table3AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg)
{
  tableAnnounceAddToList(TABLE3, msg);
}

void WaiterAgentDra::table4AnnounceCallback(const tb_waiter::DynamicRoleAssigment::ConstPtr& msg)
{
  tableAnnounceAddToList(TABLE4, msg);
}

// ###################################################################### //

void WaiterAgentDra::robotStatus(int robot_num_id, tb_waiter::DynamicRoleInfo msg)
{
  if (msg.robot_name != robot_name_)
    dra_status_list_[robot_num_id] = msg;
}

// ###################################################################### //

void WaiterAgentDra::robot0StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_0, *msg);
}

void WaiterAgentDra::robot1StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_1, *msg);
}

void WaiterAgentDra::robot2StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_2, *msg);
}

void WaiterAgentDra::robot3StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_3, *msg);
}

void WaiterAgentDra::robot4StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_4, *msg);
}

// ############################################################################################# //
// ############################################################################################# //
// ############################################################################################# //

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waiter_agent");
  WaiterAgentDra Agent;
  Agent.spin();

  return (0);
}
