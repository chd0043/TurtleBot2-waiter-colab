/*
 * kobukiBase.cpp
 *
 *  Created on: Jun 3, 2014
 *    Author: chd
 */

#include <tb_waiter/kobuki_base.h>

namespace kobuki {

kobukiBase::kobukiBase():
  ph_("~")
{
  // Get parameters
  ph_.param("turn_rate",    ang_zvel_,  2.0);
  ph_.param("forward_rate", lin_xvel_, 0.25);
  //nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  ph_.param("global_frame_id", global_frame_id_, std::string("map"));

  odom_subscriber_      = nh_.subscribe("odom",                          10, &kobukiBase::oddEventCallback,    this);
  amcl_pose_subscriber_ = nh_.subscribe("amcl_pose",                     10, &kobukiBase::amclPoseCallback,    this);
  bumper_subscriber_    = nh_.subscribe("mobile_base/events/bumper",     10, &kobukiBase::bumperEventCallback, this);
  cliff_subscriber_     = nh_.subscribe("mobile_base/events/cliff",      10, &kobukiBase::cliffEventCallback,  this);
  wheeldrop_subscriber_ = nh_.subscribe("mobile_base/events/wheel_drop", 10, &kobukiBase::wheeldropCallback,   this);
  //
  cmd_vel_publisher_    = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odom_reset_publisher_ = nh_.advertise<std_msgs::Empty>("mobile_base/commands/reset_odometry", 1);
  ini_pose_publisher_   = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  cancelgoal_publisher_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
}

kobukiBase::~kobukiBase()
{
  //
  bumper_subscriber_.shutdown();
  cliff_subscriber_.shutdown();
  odom_subscriber_.shutdown();
  amcl_pose_subscriber_.shutdown();
  wheeldrop_subscriber_.shutdown();
  cmd_vel_publisher_.shutdown();
  odom_reset_publisher_.shutdown();
  ini_pose_publisher_.shutdown();
  cancelgoal_publisher_.shutdown();
}

// ####################################################################### //

void kobukiBase::cancelMoveBaseGoal()
{
  actionlib_msgs::GoalID cancel;
  cancelgoal_publisher_.publish(cancel);
  ros::spinOnce();
  ros::Duration(0.5).sleep();
}

void kobukiBase::setInitialPose1()
{
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = ros::Time::now();
  initial_pose.header.frame_id = global_frame_id_;
  initial_pose.pose.pose.position.x = 0;
  initial_pose.pose.pose.position.y = 0;
  initial_pose.pose.pose.position.z = 0;
  initial_pose.pose.pose.orientation.x = 0;
  initial_pose.pose.pose.orientation.y = 0;
  initial_pose.pose.pose.orientation.z = 0;
  initial_pose.pose.pose.orientation.w = 1;

  ini_pose_publisher_.publish(initial_pose);
  ros::spinOnce();
}

void kobukiBase::setInitialPose2()
{
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  // TODO set a diferent initial position!
  initial_pose.header.stamp = ros::Time::now();
  initial_pose.header.frame_id = global_frame_id_;
  initial_pose.pose.pose.position.x = 0;
  initial_pose.pose.pose.position.y = 0;
  initial_pose.pose.pose.position.z = 0;
  initial_pose.pose.pose.orientation.x = 0;
  initial_pose.pose.pose.orientation.y = 0;
  initial_pose.pose.pose.orientation.z = 0;
  initial_pose.pose.pose.orientation.w = 1;

  ini_pose_publisher_.publish(initial_pose);
  ros::spinOnce();
}

// ####################################################################### //

void kobukiBase::resetOdom()
{
  std_msgs::Empty reset;
  bool reset_finished = false;
  robPosition actual_pose;
  while (reset_finished == false && ros::ok())
  {
    actual_pose = getOdomValue();
    reset_finished = ( fabs( actual_pose.x )     < 0.001 &&
                       fabs( actual_pose.y )     < 0.001 &&
                       fabs( actual_pose.theta ) < 0.001);
    odom_reset_publisher_.publish(reset);
    ros::spinOnce();
  }
  ros::Duration(0.5).sleep();
  odom_pose_ = robPosition();
  amcl_pose_ = robPosition();
  last_odom_pose_ = robPosition();
  last_amcl_pose_ = robPosition();
}

void kobukiBase::command(geometry_msgs::Twist twist)
{
  cmd_vel_publisher_.publish(twist);
  ros::spinOnce();
}

void kobukiBase::moveForward()
{
  geometry_msgs::Twist twist;
  twist.linear.x = lin_xvel_;
  twist.angular.z = 0.0;
  command(twist);
}

void kobukiBase::stopMovement()
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  command(twist);
}

void kobukiBase::moveForwardUntilBump()
{
  while (!anyBumperActivated() && ros::ok())
  {
    moveForward();
  }
  stopMovement();
}

void kobukiBase::turnLeft()
{
  // giro 90 grados a la izquiera
  //robPosition pActual = getOdomValue();
  turnController( + M_PI / 2); // pActual.theta
}

void kobukiBase::turnRigth()
{
  // giro 90 grados a la derecha
  //robPosition pActual = getOdomValue();
  turnController(- M_PI / 2); // pActual.theta
}

// ####################################################################### //

double kobukiBase::scaleVal(double oldValue, double oldMin, double oldMax, double newMin, double newMax)
{
  double oldRange, newRange, newValue;

  oldRange = (oldMax - oldMin);
  newRange = (newMax - newMin);
  newValue = ( ( (oldValue - 0) * newRange ) / oldRange ) + newMin;

  return (newValue);
}

int kobukiBase::sign(double num)
{
  if (num > 0)
    return (1);
  else
    return (-1);
  return (1);
}

bool kobukiBase::fwdController(double linear_distance)
{
  geometry_msgs::Twist twist;
  double u, max_u, accel_ramp, magnitud;
  double kp = 99999;
  double e = 1;
  double acc_step = 0.0005;
  robPosition initial_pose = getOdomValue();

  while ( ( fabs(e) > 0.05) && ros::ok() )
  {
    double dist;
    if (anyBumperActivated() || anyCliffActivated() || anyDropWheelActivated() ) return (false);

    dist =  hypot(  odom_pose_.x - initial_pose.x,  odom_pose_.y - initial_pose.y) * sign(linear_distance);
    e = linear_distance - dist;
    u = fabs(kp*e);

    if (max_u < u) max_u = u;

    if (fabs(e) < 0.25)
      magnitud = scaleVal(u, 0, max_u, 0.02, lin_xvel_);
    else
      magnitud = fabs(lin_xvel_);
    //
    twist.angular.z = 0.0;
    twist.linear.x = sign(e) * magnitud * accel_ramp;
    //ROS_INFO("e = %i, magnitud = %f, accel_ramp = %f ", sign(e),  magnitud, accel_ramp );

    command(twist);
    //rate.sleep();
    if (accel_ramp < 1)
      accel_ramp += acc_step;
    else
      accel_ramp = 1;
  }
  stopMovement();
  return (true);
}

bool kobukiBase::turnController(double theta_goal)
{
  geometry_msgs::Twist twist;
  double u, max_u, accel_ramp, angle_grad;
  double kp = 10;
  double e = 999;//0.01;
  double acc_step = 0.002;
  robPosition actual_pose = getOdomValue();

  theta_goal = actual_pose.theta + theta_goal;

  while ( ( fabs(e) > 0.009) && ros::ok() )
  {
    double e1;
    if (anyBumperActivated() || anyCliffActivated() || anyDropWheelActivated() ) return (false);

    e1 = theta_goal - odom_pose_.theta;
    e = atan2( sin(e1), cos(e1) );
    //ROS_INFO("e = %f", e);
    u = fabs(kp*e);

    if (max_u < u) max_u = u;
    double magnitud = scaleVal(u, 0, max_u, 0.2, ang_zvel_);

    twist.angular.z = sign(e) * magnitud * accel_ramp;
    command(twist);
    //rate.sleep();
    if (accel_ramp < 1)
      accel_ramp += acc_step;
    else
      accel_ramp = 1;
  }
  stopMovement();
  return (true);
}

// ####################################################################### //

void kobukiBase::moveBaseXY(robPosition goal_pose)
{
  double angle, distance, angle_grad;
  robPosition actual_pose = getOdomValue();
  ROS_INFO("PosActual: x = %f, y = %f", actual_pose.x, actual_pose.y);
  ROS_INFO("goal_pose: x = %f, y = %f", goal_pose.x, goal_pose.y);

  angle = wrapAngle( std::atan2( goal_pose.y - actual_pose.y,
                                 goal_pose.x - actual_pose.x ) - actual_pose.theta );

  distance =  hypot(goal_pose.x - actual_pose.x,
                    goal_pose.y - actual_pose.y);

  angle_grad = angle * 180 / M_PI;

  ROS_INFO("angle = %f, distance = %f", angle_grad, distance);
  turnController(angle);
  ros::Duration(0.5).sleep();
  fwdController(distance);
  ros::Duration(0.5).sleep();

  actual_pose = getOdomValue();
  ROS_INFO("Pos final: x = %f, y = %f", actual_pose.x, actual_pose.y);
}

double kobukiBase::wrapAngle(double a)
{
  a = fmod(a + M_PI, 2*M_PI);
  if (a < 0.0)
    a += 2.0*M_PI;
  return (a - M_PI);
}

// ####################################################################### //

robPosition kobukiBase::getOdomValue()
{
  ros::spinOnce();
  return (odom_pose_);
}

robPosition kobukiBase::getAmclPosition()
{
  ros::spinOnce();
  return (amcl_pose_);
}

sensor kobukiBase::getBumperState()
{
  ros::spinOnce();
  return (bumper_);
}

sensor kobukiBase::getCliffState(){
  ros::spinOnce();
  return (cliff_);
}

sensor kobukiBase::getDropWheelState()
{
  ros::spinOnce();
  return (wheel_drop_);
}

bool kobukiBase::anyBumperActivated(bool enable_spin_once)
{
  if (enable_spin_once) ros::spinOnce();
  return (bumper_.left_activated || bumper_.center_activated || bumper_.right_activated);
}

bool kobukiBase::anyCliffActivated(bool enable_spin_once)
{
  if (enable_spin_once) ros::spinOnce();
  return (cliff_.left_activated || cliff_.center_activated || cliff_.right_activated);
}

bool kobukiBase::anyDropWheelActivated(bool enable_spin_once)
{
  if (enable_spin_once) ros::spinOnce();
  return (wheel_drop_.left_activated || wheel_drop_.right_activated);
}

// ####################################################################### //
// ############################# callbacks ############################### //
// ####################################################################### //


void kobukiBase::bumperEventCallback(const kobuki_msgs::BumperEventConstPtr msg)
{
  //
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    switch (msg->bumper)
    {
    case kobuki_msgs::BumperEvent::LEFT:    bumper_.left_activated   = true;  break;
    case kobuki_msgs::BumperEvent::CENTER:  bumper_.center_activated = true;  break;
    case kobuki_msgs::BumperEvent::RIGHT:   bumper_.right_activated  = true;  break;
    }
  }
  else //
  {
    switch (msg->bumper)
    {
    case kobuki_msgs::BumperEvent::LEFT:    bumper_.left_activated   = false; break;
    case kobuki_msgs::BumperEvent::CENTER:  bumper_.center_activated = false; break;
    case kobuki_msgs::BumperEvent::RIGHT:   bumper_.right_activated  = false; break;
    }
  }
  //ROS_INFO("BumperEvent");
};

void kobukiBase::cliffEventCallback(const kobuki_msgs::CliffEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    switch (msg->sensor)
    {
    case kobuki_msgs::CliffEvent::LEFT:    cliff_.left_activated   = true;  break;
    case kobuki_msgs::CliffEvent::CENTER:  cliff_.center_activated = true;  break;
    case kobuki_msgs::CliffEvent::RIGHT:   cliff_.right_activated  = true;  break;
    }
  }
  else //
  {
    switch (msg->sensor)
    {
    case kobuki_msgs::CliffEvent::LEFT:    cliff_.left_activated   = false; break;
    case kobuki_msgs::CliffEvent::CENTER:  cliff_.center_activated = false; break;
    case kobuki_msgs::CliffEvent::RIGHT:   cliff_.right_activated  = false; break;
    }
  }
  //ROS_INFO("cliffEventCallback");
};

void kobukiBase::oddEventCallback(const nav_msgs::OdometryConstPtr& msg)
{
  //
  // Convert quaternion to RPY.
  tf::Quaternion q;
  double yaw, pitch, roll;

  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

  odom_pose_.x = msg->pose.pose.position.x;
  odom_pose_.y = msg->pose.pose.position.y;
  odom_pose_.theta = tf::getYaw(q);

  double delta_dist = hypot(odom_pose_.x - last_odom_pose_.x, odom_pose_.y - last_odom_pose_.y);
  odom_pose_.dist = last_odom_pose_.dist + delta_dist;
  last_odom_pose_ = odom_pose_;

}

void kobukiBase::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

  amcl_pose_.x = msg->pose.pose.position.x;
  amcl_pose_.y = msg->pose.pose.position.y;
  amcl_pose_.theta = tf::getYaw(q);

  double delta_dist = hypot(amcl_pose_.x - last_amcl_pose_.x, amcl_pose_.y - last_amcl_pose_.y);
  amcl_pose_.dist = last_amcl_pose_.dist + delta_dist;
  last_amcl_pose_ = amcl_pose_;
}

void kobukiBase::wheeldropCallback(const kobuki_msgs::WheelDropEventConstPtr& msg)
{
  // read wheel drops
  {
    if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
    {
      // need to keep track of both wheels separately
      if (msg->wheel == kobuki_msgs::WheelDropEvent::LEFT)
      {wheel_drop_.left_activated = true;}
      else // kobuki_msgs::WheelDropEvent::RIGHT
      {wheel_drop_.right_activated = true;}
    }
    else // kobuki_msgs::WheelDropEvent::RAISED
    {
      // need to keep track of both wheels separately
      if (msg->wheel == kobuki_msgs::WheelDropEvent::LEFT)
      {wheel_drop_.left_activated = false;}
      else // kobuki_msgs::WheelDropEvent::RIGHT
      {wheel_drop_.right_activated = false;}
    }
    //ROS_INFO("wheel_drop_");
  };

}


} /* namespace kobuki */
