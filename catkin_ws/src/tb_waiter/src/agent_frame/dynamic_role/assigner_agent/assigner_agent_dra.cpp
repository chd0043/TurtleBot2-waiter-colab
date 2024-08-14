/*
 * assigner_agent_dra.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: chd
 */

#include <tb_waiter/assigner_agent_dra.h>

AssignerAgentDra::AssignerAgentDra():
  ph_("~"),
  loop_rate_           (1),
  key_file_descriptor_ (0),
  key_                 ('\0'),
  request_memory_      (false),
  sim_monitor_request_ (false)
{
  sub_dra_status_r0_ = nh_.subscribe("/robot_0/dra_status",  10, &AssignerAgentDra::robot0StatusCallback,   this);
  sub_dra_status_r1_ = nh_.subscribe("/robot_1/dra_status",  10, &AssignerAgentDra::robot1StatusCallback,   this);
  sub_dra_status_r2_ = nh_.subscribe("/robot_2/dra_status",  10, &AssignerAgentDra::robot2StatusCallback,   this);
  sub_dra_status_r3_ = nh_.subscribe("/robot_3/dra_status",  10, &AssignerAgentDra::robot3StatusCallback,   this);
  sub_dra_status_r4_ = nh_.subscribe("/robot_4/dra_status",  10, &AssignerAgentDra::robot4StatusCallback,   this);
  sub_sim_coord_     = nh_.subscribe("sim_coordinator",      10, &AssignerAgentDra::simMonitorRequest,      this);

  pub_announce_   = nh_.advertise<tb_waiter::DynamicRoleAssigment>("dra_announce", 1);

  robot_status_.resize(5);
  robot_utilities_.resize(5);

  ph_.param("assigner_name", assigner_name_, std::string("table0"));
  ph_.param("simulation", simulation_, false); //true

  tcgetattr(key_file_descriptor_, &original_terminal_state_); // get terminal properties
  thread_.start(&AssignerAgentDra::keyboardInputLoop, *this);

  ROS_INFO("Assigner agent initialized.");
  ROS_INFO("Waiting for signal.");
}

AssignerAgentDra::~AssignerAgentDra(){}

// ######################################################################## //

void AssignerAgentDra::spin()
{
  while (ros::ok())
  {
    ros::spinOnce();
    mainProcess();
    loop_rate_.sleep();
    // ROS_INFO("une: %c", getKey() );
  }
  thread_.cancel();
  tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
}

void AssignerAgentDra::mainProcess()
{
  const geometry_msgs::PoseWithCovariance blank_pose;
  geometry_msgs::PoseWithCovariance goal;
  tb_waiter::DynamicRoleAssigment announcement;
  std::string contracted_name;

  if    (assigner_name_ == "table0"){
    goal.pose.position.x = -3.280; goal.pose.position.y = 4.460; }
    //goal.x = -3.2753636837; goal.y =  2.30509614944; }
  else if (assigner_name_ == "table1") {
    //goal.x = -5.08998632431; goal.y =  -0.326577186584; }
    goal.pose.position.x = -2.410; goal.pose.position.y = 1.010; }
  else if (assigner_name_ == "table2") {
    goal.pose.position.x = -2.492; goal.pose.position.y =-1.260; }
  else if (assigner_name_ == "table3") {
    goal.pose.position.x = -4.161; goal.pose.position.y = 5.884; }

  //
  announcement.header.stamp = ros::Time::now();
  announcement.name = assigner_name_;

  //char key = getKey();
  if ( waitUntilRequest() )//( key != '\0' )
      request_memory_ = true;

  contracted_name = selectRobot(robot_utilities_).name;
  if (request_memory_)
  {
    announcement.status = "request";
    announcement.contract = contracted_name;
    announcement.pose = goal;
  }
  else
  {
    announcement.status = "";
    announcement.contract = contracted_name;
    announcement.pose = blank_pose;
  }
  pub_announce_.publish(announcement);

}

// ###################################################

bool AssignerAgentDra::waitUntilRequest()
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

// ######################################################################### //

void AssignerAgentDra::keyboardInputLoop()
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

char AssignerAgentDra::getKey()
{
  char resp;
  resp = key_;
  key_ = (char)0;
  return(resp);
}

// ##############################################################


rbtUtilStruct AssignerAgentDra::selectRobot(std::vector<rbtUtilStruct>  robot_utilities)
{
  rbtUtilStruct assigned_robot;
  if ( robot_utilities.empty() )
  {
    ROS_ERROR("Variable 'robot_utilities' is empty.");
    return(assigned_robot);
  }

  int last = robot_utilities.size() - 1;
  std::sort(robot_utilities.begin(), robot_utilities.end());

  assigned_robot = robot_utilities[last];

  return(assigned_robot);
}


// ##############################################################

void AssignerAgentDra::robotStatus(int robot_num_id, tb_waiter::DynamicRoleInfo msg)
{
  robot_status_[robot_num_id] = msg;

  robot_utilities_[robot_num_id].name = msg.robot_name;
  robot_utilities_[robot_num_id].utility = msg.u_table[ stringToInt(assigner_name_) ];

  if ( robot_status_[robot_num_id].assigned_table == assigner_name_ &&
       robot_status_[robot_num_id].current_state == "deliver_ok" )
  {
    request_memory_ = false;
  }
}

int AssignerAgentDra::stringToInt(std::string str)
{
    std::string index = str.substr(5, str.size() );
    return( atoi( index.c_str() ) );
}

// ##############################################################

void AssignerAgentDra::robot0StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_0, *msg);
}

void AssignerAgentDra::robot1StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  //robot_status_[ROBOT_1] = *msg;
  robotStatus(ROBOT_1, *msg);
}

void AssignerAgentDra::robot2StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_2, *msg);
}

void AssignerAgentDra::robot3StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_3, *msg);
}

void AssignerAgentDra::robot4StatusCallback(const tb_waiter::DynamicRoleInfo::ConstPtr& msg)
{
  robotStatus(ROBOT_4, *msg);
}

// ##############################################################################

void AssignerAgentDra::simMonitorRequest(const std_msgs::Bool::ConstPtr& msg)
{
  sim_monitor_request_ = msg->data; //true;
  ROS_INFO("sim_monitor_request_: %i",sim_monitor_request_);
}

// ##############################################################################

int main(int argc, char **argv)
{
  ros::init(argc, argv, "assigner_dra_agent");

  AssignerAgentDra Agent;
  Agent.spin();

  return (0);
}
