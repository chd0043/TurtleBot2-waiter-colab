/*
 * KobukiCommand.cpp
 *
 *  Created on: Jul 7, 2014
 *    Author: chd
 */

#include "tb_waiter/kobuki_command.h"

// ##########################################################################################
// ##########################################################################################
// ##########################################################################################

KobukiCommand::KobukiCommand(std::string frameID):
  ph_                    ("~"),
  frameID_               (frameID),
  move_base_done_status_ (false),
  recovery_behavior_     (false),
  mbAC_                  ("move_base",        true),
  tbAC_                  ("turtlebot_move",   true),
  adAC_                  ("dock_drive_action",true)
{
  // Constructor
  ph_.param("simulation",   simulation_,  true );
  ph_.param("enable_mark_", enable_mark_, false);
  ph_.param("robot_name",   robot_name_,  std::string("robot_0"));

  //* TODO erase comments
  // waiting for Move_base action server
  while(!mbAC_.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
    ROS_INFO("Waiting for the move_base action server to come up...");
  }

  while(!tbAC_.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
    ROS_INFO("Waiting for the Turtlebot action server to come up...");
  }

  if (!simulation_)
  {
    // waiting for AutoDocking action server
    while(!adAC_.waitForServer(ros::Duration(5.0)) && ros::ok())
    {
      ROS_INFO("Waiting for the AutoDocking action server to come up...");
    }
  }
//*/
  ROS_INFO("KobukiCommand initialized...");
}

KobukiCommand::~KobukiCommand()
{
  //
  ROS_INFO("KobukiCommand destroyed...");
  cancelAllGoals(mbAC_);
  cancelAllGoals(tbAC_);
  cancelAllGoals(adAC_);

}

// #########################################################################################

template <typename T>
bool KobukiCommand::cancelAllGoals(actionlib::SimpleActionClient<T> & action_client, double timeout)
{
  actionlib::SimpleClientGoalState goal_state = action_client.getState();
  if ((goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
      (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
      (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
      (goal_state != actionlib::SimpleClientGoalState::PREEMPTED))
  {
    // Cannot cancel a REJECTED, ABORTED, SUCCEEDED or LOST goal
    ROS_WARN("Cannot cancel %s goal, as it has %s state!", acName(action_client), goal_state.toString().c_str());
    return (true);
  }

  ROS_INFO("Canceling %s goal with %s state...", acName(action_client), goal_state.toString().c_str());
  action_client.cancelAllGoals();
  if (action_client.waitForResult(ros::Duration(timeout)) == false)
  {
    ROS_WARN("Cancel %s goal didn't finish after %.2f seconds: %s",
             acName(action_client), timeout, goal_state.toString().c_str());
    return (false);
  }

  ROS_INFO("Cancel %s goal succeed. New state is %s", acName(action_client), goal_state.toString().c_str());
  return (true);
}

template <typename T>
bool KobukiCommand::waitForServer(actionlib::SimpleActionClient<T> & action_client, double timeout)
{
  // Wait for the required action servers to come up
  ros::Time t0 = ros::Time::now();

  while ((action_client.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
  {
    if ((ros::Time::now() - t0).toSec() > timeout/2.0)
      ROS_INFO(3, "Waiting for %s action server to come up...", acName(action_client));

    if ((ros::Time::now() - t0).toSec() > timeout)
    {
      ROS_ERROR("Timeout while waiting for %s action server to come up", acName(action_client));
      return (false);
    }
  }
  return (true);
}

template <typename T>
const char* KobukiCommand::acName(actionlib::SimpleActionClient<T> & action_client)
{
  return ((typeid(T) == typeid(move_base_msgs::MoveBaseAction)) ? "move base"      :
          (typeid(T) == typeid(kobuki_msgs::AutoDockingAction)) ? "auto-dock"      :
          (typeid(T) == typeid(tb_waiter::TurtlebotMoveAction)) ? "turtlebot-move" : "ERROR");
}

// ##########################################################################################
// #################### Move_base actions ###################################################
// ##########################################################################################

void KobukiCommand::cancelGoal()
{
  mbAC_.cancelGoal();
  mbAC_.waitForResult();
}

void KobukiCommand::navigateToGoals(std::vector<RobPosition> goals)
{
  for (std::vector<RobPosition>::iterator it = goals.begin() ; it != goals.end(); ++it){
    navigateToGoal(*it);
  }
}

bool KobukiCommand::navigateToGoal(RobPosition goal)
{
  bool bMovementFinished = false;
  MoveBaseGoal move_base_goal = createMoveBaseGoal(goal.x,goal.y,frameID_);
  move_base_done_status_ = false;

  RobPosition actual_pose = getPosActual();
  double theta = wrapAngle( std::atan2( goal.y - actual_pose.y,
                                        goal.x - actual_pose.x ) - actual_pose.theta );
  //double to_turn = heading(position, goal);
  turtlebotMove(theta, 0);

  while (!bMovementFinished && ros::ok()){
    ROS_INFO("Sending goal");
    mbAC_.sendGoal(move_base_goal,
        boost::bind(&KobukiCommand::moveBaseDoneCb,     this, _1, _2),
        boost::bind(&KobukiCommand::moveBaseActiveCb,   this),
        boost::bind(&KobukiCommand::moveBaseFeedbackCb, this, _1));
        //mbAC_.waitForResult();
        waitForMoveBaseResult();
    if(mbAC_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("rotation reached.");
      bMovementFinished = true;
      return (bMovementFinished);
    }
    else
    {
      ROS_ERROR("Could not execute rotation to goal for some reason.");
    }
  }
  return (bMovementFinished);
}

void KobukiCommand::moveBaseDoneCb(const actionlib::SimpleClientGoalState& status,
                                   const move_base_msgs::MoveBaseResultConstPtr & result)
{
  //if(status.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_DEBUG("Result - [move_base ActionServer: %s ]", status.toString().c_str());
  move_base_done_status_ = true;
}

void KobukiCommand::moveBaseActiveCb()
{
  ROS_DEBUG("Action server went active");
}

void KobukiCommand::moveBaseFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr & feedback)
{
  double x = feedback->base_position.pose.position.x;
  double y = feedback->base_position.pose.position.y;
}

void KobukiCommand::sendMoveBaseGoal(RobPosition goal)
{
  MoveBaseGoal move_base_goal = createMoveBaseGoal(goal.x,goal.y,frameID_);
  move_base_done_status_ = false;

  RobPosition actual_pose = getPosActual();
  double theta = wrapAngle( std::atan2( goal.y - actual_pose.y,
                                        goal.x - actual_pose.x ) - actual_pose.theta );
  //double to_turn = heading(position, goal);
  turtlebotMove(theta, 0);

  ROS_INFO("Sending goal");
  mbAC_.sendGoal(move_base_goal,
         boost::bind(&KobukiCommand::moveBaseDoneCb,     this, _1, _2),
         boost::bind(&KobukiCommand::moveBaseActiveCb,   this),
         boost::bind(&KobukiCommand::moveBaseFeedbackCb, this, _1));
}

MoveBaseGoal KobukiCommand::createMoveBaseGoal(double x, double y, std::string frame_id)
{
  MoveBaseGoal goal;
  double theta = calcRotationAngle(x,y);
  //
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta);
  return (goal);
}

double KobukiCommand::calcRotationAngle(double x, double y)
{
  double angle,xd,yd;
  RobPosition position = getPosActual();

  xd = x - position.x;
  yd = y - position.y;
  angle = std::atan2(yd,xd);
  return (angle);
}

bool KobukiCommand::waitForMoveBaseResult()
{
  while (!move_base_done_status_ && ros::ok() )
  {
    ros::spinOnce();
  }
  return( true );
}

void KobukiCommand::waitForMoveBaseResult(const ros::Duration& timeout)
{
    mbAC_.waitForResult(timeout);
}

bool KobukiCommand::getMoveBaseDoneStatus()
{
  return(move_base_done_status_);
}

double KobukiCommand::wrapAngle(double a)
{
  a = fmod(a + M_PI, 2*M_PI);
  if (a < 0.0)
    a += 2.0*M_PI;
  return (a - M_PI);
}

template <typename S,typename T>
double KobukiCommand::heading(const S &a, const T &b)
{
  double a_x, a_y, b_x, b_y;
  if (typeid(S) == typeid(tf::Vector3)  )  { a_x = a.x();             a_y = a.y();             }
  if (typeid(S) == typeid(tf::Transform))  { a_x = a.getOrigin().x(); a_y = a.getOrigin().y(); }
  if (typeid(S) == typeid(RobPosition)  )  { a_x = a.x;               a_y = a.y;               }
  if (typeid(T) == typeid(tf::Vector3)  )  { b_x = b.x();             a_y = a.y();             }
  if (typeid(T) == typeid(tf::Transform))  { b_x = b.getOrigin().x(); a_y = b.getOrigin().y(); }
  if (typeid(T) == typeid(RobPosition)  )  { b_x = b.x;               a_y = a.y;               }

return ( std::atan2( b_y - a_y, b_x - a_x ) );
}
// TODO
/*
double KobukiCommand::heading(const tf::Vector3& a, const tf::Vector3& b)
{
  return ( std::atan2(b.y() - a.y(), b.x() - a.x()) );
}

double KobukiCommand::heading(const tf::Transform& a, const tf::Transform& b)
{
  return ( heading(a.getOrigin(), b.getOrigin()) );
}

double KobukiCommand::heading(const RobPosition& a, const RobPosition& b)
{
  tf::Vector3 a2( tfScalar(a.x), tfScalar(a.y), tfScalar(0) );
  tf::Vector3 b2( tfScalar(b.x), tfScalar(b.y), tfScalar(0) );
  return ( heading(a, b) );
}
*/

bool KobukiCommand::clearCostmaps()
{
  ros::Time t0 = ros::Time::now();

  ros::NodeHandle nh;
  ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  std_srvs::Empty srv;

  if (client.call(srv) == true)
  {
    ROS_INFO("Successfully cleared costmaps (%f seconds)", (ros::Time::now() - t0).toSec());
    return (true);
  }
  else
  {
    ROS_ERROR("Failed to clear costmaps (%f seconds)", (ros::Time::now() - t0).toSec());
    return (false);
  }
}

bool KobukiCommand::cleanupAndError()
{
  clearCostmaps();
//  disableSafety();
  enableRecovery();
  cancelAllGoals(mbAC_);
  cancelAllGoals(tbAC_);
  cancelAllGoals(adAC_);

  return (false);
}

bool KobukiCommand::enableRecovery()
{
//  if (recovery_behavior_ == true)
//    return (true);

  ros::Time t0 = ros::Time::now();
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/set_parameters");
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "recovery_behavior_enabled";
  srv.request.config.bools[0].value = true;
//  srv.request.config.doubles.resize(1);
//  srv.request.config.doubles[0].name = "planner_frequency";
//  srv.request.config.doubles[0].value = default_planner_frequency_;

  if (client.call(srv) == true)
  {
    ROS_INFO("Recovery behavior enabled (%f seconds)", (ros::Time::now() - t0).toSec());
    //recovery_behavior_ = true;
    return (true);
  }
  else
  {
    ROS_ERROR("Failed to enable recovery behavior (%f seconds)", (ros::Time::now() - t0).toSec());
    return (false);
  }

  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behavior_enabled: true }\"");  // clearing_rotation_allowed
  if (status != 0)
  {
    ROS_ERROR("Enable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return (false);
  }
  ROS_DEBUG("enableRecovery ended on %f s", (ros::Time::now() - t0).toSec());
  //recovery_behavior_ = true;
  return (true);
}

bool KobukiCommand::disableRecovery()
{
  //if (recovery_behavior_ == false)
  //  return true;

  ros::Time t0 = ros::Time::now();
  ROS_DEBUG("disableRecovery starts....");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/set_parameters");
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "recovery_behavior_enabled";
  srv.request.config.bools[0].value = false;

  if (client.call(srv) == true)
  {
    ROS_INFO("Recovery behavior disabled (%f seconds)", (ros::Time::now() - t0).toSec());
    //recovery_behavior_ = false;
    return (true);
  }
  else
  {
    ROS_ERROR("Failed to disable recovery behavior (%f seconds)", (ros::Time::now() - t0).toSec());
    return (false);
  }

  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behavior_enabled: false }\"");  // clearing_rotation_allowed
  if (status != 0)
  {
    ROS_ERROR("Disable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return (false);
  }
  ROS_DEBUG("disableRecovery ended on %f", (ros::Time::now() - t0).toSec());
  //recovery_behavior_ = false;
  return (true);
}

bool KobukiCommand::softRecovery()
{
  if (recovery_behavior_ == true)
    return (true);

  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behaviors: [{name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}] }\"");  // recovery_behavior_enabled
  if (status != 0)
  {
    ROS_ERROR("Enable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return (false);
  }

  recovery_behavior_ = true;
  return (true);
}

bool KobukiCommand::hardRecovery()
{
  if (recovery_behavior_ == false)
    return (true);

  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behaviors: [{name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}, {name: rotate_recovery, type: rotate_recovery/RotateRecovery}, {name: aggressive_reset, type: clear_costmap_recovery/ClearCostmapRecovery}] }\"");  // recovery_behavior_enabled
  if (status != 0)
  {
    ROS_ERROR("Disable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return (false);
  }

  recovery_behavior_ = false;
  return (true);
}

tf::StampedTransform KobukiCommand::getTf(const std::string& frame_1, const std::string& frame_2)
{
  tf::StampedTransform tf;
  try
  {
    tf_listener_.lookupTransform(frame_1, frame_2, ros::Time(0.0), tf);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Cannot get tf %s -> %s: %s", frame_1.c_str(), frame_1.c_str(), e.what());
  }
  return (tf);
}

tf::StampedTransform KobukiCommand::getOdomTf()
{
  // Get latest map -> odom tf
  return ( getTf(global_frame_, odom_frame_) );
}

tf::StampedTransform KobukiCommand::getRobotTf()
{
  // Get latest robot global pose
  return ( getTf(global_frame_, base_frame_) );
}

// ##########################################################################################
// #################### Turtlebot actions ###################################################
// ##########################################################################################

bool KobukiCommand::turtlebotMove(double turn_distance, double forward_distance)
{
  tb_waiter::TurtlebotMoveGoal action_goal;

  action_goal.turn_distance = turn_distance;
  action_goal.forward_distance = forward_distance;

  ROS_INFO("send action goal.");
  tbAC_.sendGoal(action_goal);

  //wait for the action to return
  bool finished_before_timeout = tbAC_.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = tbAC_.getState();
    ROS_INFO("Turtlebot Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Turtlebot Action did not finish before the time out.");
  }
  return(finished_before_timeout);
}

void KobukiCommand::turnLeft()
{
  double turn_distance = M_PI/2;
  double forward_distance = 0;
  turtlebotMove(turn_distance,forward_distance);
}

void KobukiCommand::turnRigth()
{
  double turn_distance = -M_PI/2;
  double forward_distance = 0;
  turtlebotMove(turn_distance,forward_distance);
}

void KobukiCommand::turnAngle(double turn_distance)
{
    double forward_distance = 0;
    turtlebotMove(turn_distance,forward_distance);
}

void KobukiCommand::moveForward(double forward_distance)
{
  double turn_distance = 0;
  turtlebotMove(turn_distance,forward_distance);
}

void KobukiCommand::moveBaseXY(RobPosition goal_pose)
{
  double angle, distance, angle_grad;
  tf::StampedTransform robot_gb, goal_gb;
  RobPosition actual_pose = getPosActual();

  //robot_gb = getRobotTf();
  ROS_INFO("actual_pose: x = %f, y = %f, th= %f", actual_pose.x, actual_pose.y, actual_pose.theta*180 / M_PI);
  ROS_INFO("goal_pose: x = %f, y = %f", goal_pose.x, goal_pose.y);

  //angle = wrapAngle(heading(actual_pose, goal_pose) - actual_pose.theta);

  angle = wrapAngle( std::atan2( goal_pose.y - actual_pose.y,
                                 goal_pose.x - actual_pose.x ) - actual_pose.theta );

  distance =  hypot(goal_pose.x - actual_pose.x,
                    goal_pose.y - actual_pose.y);

  angle_grad = angle * 180 / M_PI;

  ROS_INFO("angle = %f, distance = %f", angle_grad, distance);
  turnAngle(angle);
  ros::Duration(0.5).sleep();
  moveForward(distance);
  ros::Duration(0.5).sleep();

  actual_pose = getPosActual();
  ROS_INFO("actual_pose: x = %f, y = %f, th= %f", actual_pose.x, actual_pose.y, actual_pose.theta*180 / M_PI);
}

// ##########################################################################################
// #################### Turtlebot services ##################################################
// ##########################################################################################

RobPosition KobukiCommand::getPosActual()
{
  return ( readRobPos("amcl") );
}

RobPosition KobukiCommand::readRobPos(std::string infoSource)
{
  RobPosition pos;
  while (!ros::service::waitForService("turtlebot_service/robPosInfo", ros::Duration(0.5)))
  {
    ROS_INFO("Waiting for service '~robPosInfo' to become available");
  }
  ros::ServiceClient client = nh_.serviceClient<tb_waiter::robPosInfo>("turtlebot_service/robPosInfo");
  tb_waiter::robPosInfo srv;
  srv.request.topic = infoSource;

  if (client.call(srv))
  {
    pos.x = srv.response.x;
    pos.y = srv.response.y;
    pos.theta = srv.response.theta;
    pos.dist = srv.response.dist;
  }
  else
  {
    ROS_ERROR("Failed to call service robPosInfo");
  }
  return (pos);
}

double KobukiCommand::estimateCost(RobPosition goal)
{
  double vel, distance;
  while (!ros::service::waitForService("turtlebot_service/robPlanDist", ros::Duration(0.5)))
  {
    ROS_INFO("Waiting for service '~robPlanDist' to become available");
  }

  ros::ServiceClient client = nh_.serviceClient<tb_waiter::robPlanDist>("turtlebot_service/robPlanDist");
  tb_waiter::robPlanDist srv;
  srv.request.x = goal.x;
  srv.request.y = goal.y;

  if (client.call(srv))
  {
    distance = srv.response.dist;
    ROS_INFO("distance: %f", distance);
  }
  else
  {
    ROS_ERROR("Failed to call service robPlanDist");
  }
  if (ros::param::has("move_base/TrajectoryPlannerROS/max_vel_x"))
  {
    nh_.getParam("move_base/TrajectoryPlannerROS/max_vel_x", vel);
  }
  else
  {
    vel = 1;
  }
  return ( distance / vel );
}

double KobukiCommand::getMeasuredDistance()
{
  return (readRobPos("amcl").dist );
}

void KobukiCommand::resetOdom()
{
  kobukiCommand("reset_Odom");
}

void KobukiCommand::setInitialPose(int num)
{
  if ( num == 1 )
  {
    kobukiCommand("initialpose1");
  }
  else if ( num == 2 )
  {
    kobukiCommand("initialpose2");
  }
  else
  {
    ROS_WARN("Uknown InitialPose");
  }
}

void KobukiCommand::kobukiCommand(std::string command)
{
  while (!ros::service::waitForService("turtlebot_service/robResetPos", ros::Duration(0.5)))
  {
    ROS_INFO("Waiting for service '~robResetPos' to become available");
  }
  ros::ServiceClient client = nh_.serviceClient<tb_waiter::robResetPos>("turtlebot_service/robResetPos");
  tb_waiter::robResetPos srv;
  srv.request.command = command;

  if (client.call(srv))
  {
    int status = srv.response.ok;
  }
  else
  {
    ROS_ERROR("Failed to call service robResetPos.");
  }
}

// ##########################################################################################
// ######################### Mark recognition service  ######################################
// ##########################################################################################

cvLandMark KobukiCommand::getLandMark(std::string command)
{
  cvLandMark landMark;
  while (!ros::service::waitForService("cvMarkRec", ros::Duration(3.0)))
  {
    ROS_INFO("Waiting for service '~cvMarkRec' to become available");
  }
  ros::ServiceClient client =nh_.serviceClient<tb_waiter::cvMarkRec>("cvMarkRec");
  tb_waiter::cvMarkRec srv;
  srv.request.command = command;

  if (client.call(srv))
  {
    landMark.figure = srv.response.figure;
    landMark.color = srv.response.color;
  }
  else
  {
    ROS_ERROR("Failed to call service cvMarkRec");
  }

  return (landMark);
}

int KobukiCommand::readRefLandMark()
{
  int refPosition;
  cvLandMark mark = getLandMark();
  ROS_INFO("");
  if   ( mark.figure == "triangle" )
    refPosition = 1;
  else if ( mark.figure == "circle" )
    refPosition = 2;
  else
    refPosition = 1;
  return (refPosition);
}

// ########################################################################################## //
// ######################### Auto Docking  ################################################## //
// ########################################################################################## //

void KobukiCommand::dockDoneCb(const actionlib::SimpleClientGoalState& status,
    const kobuki_msgs::AutoDockingResultConstPtr & result)
{
  ROS_INFO("Result - [ActionServer: %s ]: %s", status.toString().c_str(), result->text.c_str() );
}

void KobukiCommand::dockActiveCb()
{
  ROS_INFO("Action server went active");
}

void KobukiCommand::dockFeedbackCb(const kobuki_msgs::AutoDockingFeedbackConstPtr & feedback)
{
  ROS_INFO("Feedback: [DockDrive:  %s ]: %s", feedback->state.c_str(), feedback->text.c_str() );
}

void KobukiCommand::dockDriveClient()
{
  kobuki_msgs::AutoDockingGoal action_goal;

  adAC_.sendGoal(action_goal,
      boost::bind(&KobukiCommand::dockDoneCb,     this, _1, _2),
      boost::bind(&KobukiCommand::dockActiveCb,   this),
      boost::bind(&KobukiCommand::dockFeedbackCb, this, _1));
  ROS_INFO("Goal: Sent.");
  adAC_.waitForResult();
  adAC_.getResult();
}

void KobukiCommand::dockingSequence()
{
  dockDriveClient();
}

// ############################################################## //

bool KobukiCommand::rAutodocking()
{
  int refPosition;
  RobPosition pose_initial;

  if (simulation_)
      return (true);

  ROS_INFO("Waiting to be autodocked...");
  dockingSequence();

  // Set initial position
  ROS_INFO("Set initial pose.");

  if (enable_mark_)
    refPosition = readRefLandMark();
  else
    refPosition = 1;
  setInitialPose(refPosition);
  ROS_INFO("Initialpose %i was set.",refPosition);

  ros::Duration(0.5).sleep();
  moveForward(-0.15);
  ros::Duration(0.5).sleep();
  //
  if (robot_name_ == "robot_0")
  {
    pose_initial.x=-0.5;
    pose_initial.y=0.5;
  }
  else if (robot_name_ == "robot_1")
  {
    pose_initial.x=-0.5;
    pose_initial.y=-0.5;
  }
  else if (robot_name_ == "robot_2")
  {
    pose_initial.x=-0.5;
    pose_initial.y=0.5;
  }
  else if (robot_name_ == "robot_3")
  {
    pose_initial.x=-0.5;
    pose_initial.y=-0.5;
  }

  turnLeft();
  turnLeft();
  ros::Duration(0.5).sleep();

  moveBaseXY(pose_initial);
  return (true);
}

