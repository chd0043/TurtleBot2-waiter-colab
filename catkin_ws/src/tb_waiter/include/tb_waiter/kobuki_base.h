/*
 * kobukiBase.h
 *
 *  Created on: Jun 3, 2014
 *      Author: chd
 */

#ifndef KOBUKIBASE_H_
#define KOBUKIBASE_H_

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <actionlib_msgs/GoalID.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <ros/param.h>
#include <boost/foreach.hpp>

// ####################################################################### //

// topicos
const std::string CMD_TOPIC_SIM = "cmd_vel";
const std::string CMD_TOPIC = "mobile_base/commands/velocity";

namespace kobuki {

struct robPosition {
    double x;
    double y;
    double theta;
    double dist;
    robPosition():x(0),y(0),
                  theta(0), dist(0) {}
};

struct sensor {
    bool left_activated;
    bool center_activated;
    bool right_activated;
    sensor():left_activated(false),
             center_activated(false),
             right_activated(false) {}
};

// ####################################################################### //


class kobukiBase {

public:
	kobukiBase();
	virtual ~kobukiBase();

	// ####################################################################### //
    bool fwdController(double linear_distance);
    bool turnController(double theta_goal);
    void moveBaseXY(robPosition goal_pose);
    void moveForward();
    void stopMovement();
    void moveForwardUntilBump();
    void turnLeft();
    void turnRigth();
    void resetOdom();
    void setInitialPose1();
    void setInitialPose2();
    void cancelMoveBaseGoal();

	robPosition getOdomValue();
	robPosition getAmclPosition();
	sensor getBumperState();
	sensor getCliffState();
	sensor getDropWheelState();
	bool anyBumperActivated(bool enable_spin_once=false);
	bool anyCliffActivated(bool enable_spin_once=false);
	bool anyDropWheelActivated(bool enable_spin_once=false);

	// ####################################################################### //
	robPosition odom_pose_, amcl_pose_, last_odom_pose_, last_amcl_pose_;
	sensor bumper_, cliff_, wheel_drop_;
	std::string global_frame_id_;

private:
	// ####################################################################### //
    void bumperEventCallback(const kobuki_msgs::BumperEventConstPtr msg);
    void cliffEventCallback(const kobuki_msgs::CliffEventConstPtr msg);
    void oddEventCallback(const nav_msgs::OdometryConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void wheeldropCallback(const kobuki_msgs::WheelDropEventConstPtr& msg);

    int sign(double num);
    double wrapAngle(double a);
    double scaleVal(double oldValue, double oldMin, double oldMax, double newMin, double newMax);
    void command(geometry_msgs::Twist twist);

    // ####################################################################### //
    ros::Subscriber bumper_subscriber_, cliff_subscriber_, odom_subscriber_,
                    wheeldrop_subscriber_, amcl_pose_subscriber_;
    ros::Publisher cmd_vel_publisher_, odom_reset_publisher_, ini_pose_publisher_,
                   cancelgoal_publisher_;
    geometry_msgs::Twist vel_;

    double lin_xvel_, ang_zvel_;

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
};

} /* namespace kobuki */
#endif /* KOBUKIBASE_H_ */
